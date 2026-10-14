#include "buffer_backup.h"

#include <algorithm>
#include <cstring>

using namespace daisy;
using beads::Processor;

// QSPI layout: the settings sector is at 0x400000, the backup takes the next
// 12 64K blocks: one 256 byte page for the header, then the buffer.
const uint32_t kBackupQspiOffset = 0x410000;
const uint32_t kBlockSize = 0x10000;
const uint32_t kHeaderSize = 256;
const size_t kBufferSize = beads::kDelayBufferSizeBits / 8;
const size_t kNumBlocks = (kHeaderSize + kBufferSize + kBlockSize - 1) \
    / kBlockSize;

// Divides the 256K delay banks, so a chunk never straddles two of them.
const size_t kChunkSize = 4096;

const uint32_t kTag = 0x5a5a5246;  // FRZZ, like Beads
const uint32_t kBackupDelay = 10000;  // ms, like Beads

void BufferBackup::Init(QSPIHandle* qspi, Processor* processor) {
  qspi_ = qspi;
  processor_ = processor;
  state_ = STATE_IDLE;
  started_ = false;
  freeze_start_time_ = System::GetNow();

  const Header* header = static_cast<const Header*>(
      qspi_->GetData(kBackupQspiOffset));
  saved_ = header->tag == kTag && header->media <= Processor::MEDIA_DIGITAL;
}

bool BufferBackup::Load(bool frozen) {
  if (!saved_) {
    return false;
  }
  const Header* header = static_cast<const Header*>(
      qspi_->GetData(kBackupQspiOffset));
  if (!frozen) {
    // Freeze was turned off while the module was off.
    Invalidate();
    return false;
  }

  processor_->Configure(Processor::Media(header->media), header->mono);
  const uint8_t* data = static_cast<const uint8_t*>(
      qspi_->GetData(kBackupQspiOffset + kHeaderSize));
  for (size_t offset = 0; offset < kBufferSize; offset += kChunkSize) {
    std::memcpy(
        processor_->delay_buffer_raw_bytes(offset),
        &data[offset],
        std::min(kChunkSize, kBufferSize - offset));
  }
  *processor_->mutable_delay_write_ptr() = header->write_ptr;

  // Don't back up the restored buffer again.
  saved_ = true;
  started_ = true;
  return true;
}

void BufferBackup::Invalidate() {
  // Programming can only clear bits, so zeroing the tag needs no erase.
  uint32_t zero = 0;
  qspi_->Write(
      kBackupQspiOffset,
      sizeof(zero),
      reinterpret_cast<uint8_t*>(&zero));
  saved_ = false;
}

int BufferBackup::progress() const {
  // Erasing counts for the first half, writing for the second.
  if (state_ == STATE_ERASING) {
    return int(50 * erased_blocks_ / kNumBlocks);
  } else if (state_ == STATE_WRITING) {
    return int(50 + 50 * written_bytes_ / kBufferSize);
  }
  return saved_ ? 100 : 0;
}

void BufferBackup::Process(bool frozen) {
  if (!frozen) {
    if (state_ != STATE_IDLE) {
      // The erase already destroyed the header, nothing to invalidate.
      state_ = STATE_IDLE;
    } else if (saved_) {
      Invalidate();
    }
    started_ = false;
    freeze_start_time_ = System::GetNow();
    return;
  }

  if (!started_ && System::GetNow() - freeze_start_time_ >= kBackupDelay) {
    started_ = true;
    saved_ = false;
    state_ = STATE_ERASING;
    erased_blocks_ = 0;
    written_bytes_ = 0;
    header_.tag = kTag;
    header_.write_ptr = *processor_->mutable_delay_write_ptr();
    header_.media = processor_->media();
    header_.mono = processor_->mono();
  }

  // A quality mode change reinterprets the buffer, give up on this copy.
  if (state_ != STATE_IDLE && processor_->media() != header_.media) {
    state_ = STATE_IDLE;
    return;
  }

  if (state_ == STATE_ERASING) {
    uint32_t address = kBackupQspiOffset + erased_blocks_ * kBlockSize;
    qspi_->Erase(address, address + kBlockSize);
    if (++erased_blocks_ == kNumBlocks) {
      state_ = STATE_WRITING;
    }
  } else if (state_ == STATE_WRITING) {
    size_t size = std::min(kChunkSize, kBufferSize - written_bytes_);
    qspi_->Write(
        kBackupQspiOffset + kHeaderSize + written_bytes_,
        size,
        processor_->delay_buffer_raw_bytes(written_bytes_));
    written_bytes_ += size;
    if (written_bytes_ == kBufferSize) {
      // Written last: from now on the backup counts as complete.
      qspi_->Write(
          kBackupQspiOffset,
          sizeof(header_),
          reinterpret_cast<uint8_t*>(&header_));
      state_ = STATE_IDLE;
      saved_ = true;
    }
  }
}

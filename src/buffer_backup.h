// bead.init: keeps a frozen buffer across power cycles, like Beads'
// BackgroundBufferBackup, but in the Daisy's QSPI flash.
//
// After freeze has been on for 10 seconds, the delay buffer is copied to QSPI
// a little at a time from main(). The header is written last and marks the
// backup as complete. Unfreezing aborts a copy in progress and invalidates a
// complete backup by zeroing its tag, which needs no erase.

#ifndef BEAD_INIT_BUFFER_BACKUP_H_
#define BEAD_INIT_BUFFER_BACKUP_H_

#include "daisy_patch_sm.h"

#include "beads/dsp/processor.h"

class BufferBackup {
 public:
  BufferBackup() { }
  ~BufferBackup() { }

  void Init(daisy::QSPIHandle* qspi, beads::Processor* processor);

  // Restores a complete backup into the processor (media, mono, buffer and
  // write pointer) if freeze is on at power-up, and returns true. A backup
  // found with freeze off is invalidated.
  bool Load(bool frozen);

  // Call from main(). Erases one 64K block or writes one chunk per call.
  void Process(bool frozen);

  inline bool busy() const { return state_ != STATE_IDLE; }
  inline bool saved() const { return saved_; }
  // 0-100 while busy.
  int progress() const;

 private:
  enum State {
    STATE_IDLE,
    STATE_ERASING,
    STATE_WRITING
  };

  struct Header {
    uint32_t tag;
    int32_t write_ptr;
    uint8_t media;
    uint8_t mono;
  };

  void Invalidate();

  daisy::QSPIHandle* qspi_;
  beads::Processor* processor_;

  State state_;
  bool saved_;
  bool started_;
  uint32_t freeze_start_time_;
  size_t erased_blocks_;
  size_t written_bytes_;
  Header header_;
};

#endif  // BEAD_INIT_BUFFER_BACKUP_H_

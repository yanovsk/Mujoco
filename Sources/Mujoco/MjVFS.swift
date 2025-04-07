import C_mujoco
import Foundation

public final class MjVFS {
  let _vfs: UnsafeMutablePointer<mjVFS>

  public init(assets: [String: Data]) {
    _vfs = UnsafeMutablePointer.allocate(capacity: 1)
    mj_defaultVFS(_vfs)

    for (filename, data) in assets {
      filename.withCString { cStr in
        data.withUnsafeBytes { rawBuffer in
          let result = mj_addBufferVFS(_vfs, cStr, rawBuffer.baseAddress, Int32(data.count))
          if result != 0 {
            print("Error adding file \(filename) to VFS: \(result)")
          }
        }
      }
    }
  }

  deinit {
    mj_deleteVFS(_vfs)
    _vfs.deallocate()
  }
}

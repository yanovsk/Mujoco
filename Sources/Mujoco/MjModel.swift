import C_mujoco
import Foundation

public final class MjModel {
  let _model: UnsafeMutablePointer<mjModel>

  init(model: UnsafeMutablePointer<mjModel>) {
    _model = model
  }

  deinit {
    mj_deleteModel(_model)
  }

  public convenience init?(fromBinaryPath filePath: String) {
    guard let model = mj_loadModel(filePath, nil) else {
      return nil
    }
    self.init(model: model)
  }

  public convenience init?(fromXMLPath filePath: String) {
    guard let model = mj_loadXML(filePath, nil, nil, 0) else {
      return nil
    }
    self.init(model: model)
  }
    
    //in assets keys must be filenames
    public convenience init(fromXML: String, assets: [String: Data]) throws {
      var xmlString = fromXML  // Make a mutable copy.
      let errorStr = UnsafeMutablePointer<CChar>.allocate(capacity: 256)
      defer { errorStr.deallocate() }
      
      let model: UnsafeMutablePointer<mjModel>? = xmlString.withUTF8 { utf8 in
        let vfs = MjVFS(assets: assets)
        
        // Generate a unique filename for the XML file.
        var modelName = "model_"
        while assets[modelName + ".xml"] != nil {
          modelName += "_"
        }
        let fileName = "\(modelName).xml"
        
        // Add the XML as a buffer into the VFS under fileName.
        fileName.withCString { cStr in
          let result = mj_addBufferVFS(vfs._vfs, cStr, utf8.baseAddress, Int32(utf8.count))
          if result != 0 {
            print("Failed to add XML to VFS. Error code: \(result)")
          }
        }
        
        // Load the model from the XML file in the VFS.
        return mj_loadXML(fileName, vfs._vfs, errorStr, 256)
      }
      
      guard let model = model else {
        let error = String(cString: errorStr)
        throw NSError(domain: "MuJoCo XML error", code: 1, userInfo: [NSLocalizedDescriptionKey: error])
      }
      
      self.init(model: model)
    }

    
  public func makeData() -> MjData {
    let data = mj_makeData(_model)!
    return MjData(data: data, nq: _model.pointee.nq, nv: _model.pointee.nv, na: _model.pointee.na, nu: _model.pointee.nu, nbody: _model.pointee.nbody, nmocap: _model.pointee.nmocap, nuserdata: _model.pointee.nuserdata, nsensordata: _model.pointee.nsensordata)
  }

  public func step(data: MjData) {
    mj_step(_model, data._data)
  }

  public func step1(data: MjData) {
    mj_step1(_model, data._data)
  }

  public func step2(data: MjData) {
    mj_step2(_model, data._data)
  }

  public func forward(data: MjData) {
    mj_forward(_model, data._data)
  }

  public func inverse(data: MjData) {
    mj_inverse(_model, data._data)
  }

  public func forwardSkip(data: MjData, skipStage: Int32, skipSensor: Int32) {
    mj_forwardSkip(_model, data._data, skipStage, skipSensor)
  }

  public func inverseSkip(data: MjData, skipStage: Int32, skipSensor: Int32) {
    mj_inverseSkip(_model, data._data, skipStage, skipSensor)
  }

  public func reset(data: MjData) {
    mj_resetData(_model, data._data)
  }

  public func reset(data: MjData, keyframe: Int32) {
    mj_resetDataKeyframe(_model, data._data, keyframe)
  }
    
    public var nbody: Int {
        return Int(_model.pointee.nbody)
    }

    public var njnt: Int {
        return Int(_model.pointee.njnt)
    }

    public var ngeom: Int {
        return Int(_model.pointee.ngeom)
    }

    public func bodyName(atIndex i: Int) -> String {
        // Step 1: find the offset in the `names` buffer
        let offset = Int(_model.pointee.name_bodyadr[i])
        
        // Step 2: get a pointer to that C string
        let basePtr = _model.pointee.names! // 'names' is a char* in the mjModel
        let cStringPtr = basePtr.advanced(by: offset)
        
        // Step 3: convert C string -> Swift String
        return String(cString: cStringPtr)
    }
    
    public func jointName(atIndex i: Int) -> String {
        let offset = Int(_model.pointee.name_jntadr[i])
        let basePtr = _model.pointee.names!
        let cStringPtr = basePtr.advanced(by: offset)
        return String(cString: cStringPtr)
    }
    

  // Initial State.
  var qpos0: MjNumArray {
    get {
      MjNumArray(array: _model.pointee.qpos0, object: self, len: _model.pointee.nq)
    }
    set {
      guard _model.pointee.qpos0 != newValue._array else { return }
      _model.pointee.qpos0.assign(from: newValue._array, count: Int(_model.pointee.nq))
    }
  }

  var qpos_spring: MjNumArray {
    get {
      MjNumArray(array: _model.pointee.qpos_spring, object: self, len: _model.pointee.nq)
    }
    set {
      guard _model.pointee.qpos_spring != newValue._array else { return }
      _model.pointee.qpos_spring.assign(from: newValue._array, count: Int(_model.pointee.nq))
    }
  }
}

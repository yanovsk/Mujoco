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
    
    
    public func bodyId(named name: String) -> Int? {
           let id = name.withCString { cname in
               mj_name2id(_model, Int32(mjOBJ_BODY.rawValue), cname)
           }
           return id >= 0 ? Int(id) : nil
       }
    
    public func jointName(atIndex i: Int) -> String {
        let offset = Int(_model.pointee.name_jntadr[i])
        let basePtr = _model.pointee.names!
        let cStringPtr = basePtr.advanced(by: offset)
        return String(cString: cStringPtr)
    }
    
    
    public func keyframeId(named name: String) -> Int32? {
        let keyType = Int32(mjOBJ_KEY.rawValue) // Convert mjtObj_ to Int32
        let keyframeId = name.withCString { cname in
            mj_name2id(_model, keyType, cname)
        }
        return keyframeId >= 0 ? keyframeId : nil
    }
    
    public func mocapId(forBodyNamed name: String) -> Int? {
        guard let bodyId = self.bodyId(named: name) else { return nil }
        let id = _model.pointee.body_mocapid[bodyId]
        return id >= 0 ? Int(id) : nil
    }
    
    
    public func frameId(named name: String, type: Int32) -> Int? {
        let id = name.withCString { cname in
            mj_name2id(_model, type, cname)
        }
        return id >= 0 ? Int(id) : nil
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
    
    public func mat2Quat(_ xmat: [Double]) -> [Double] {
        precondition(xmat.count == 9)
        var quat = [Double](repeating: 0.0, count: 4)
        var mat = xmat  // Mujoco wants mutable
        mju_mat2Quat(&quat, &mat)
        return quat
    }
    
    public func mjKnematics(data: MjData) {
        mj_kinematics(_model, data._data)
    }

    public func mjComPos(data: MjData) {
        mj_comPos(_model, data._data)
    }
    
    public func getNq() -> Int32 {
        return _model.pointee.nq
    }
    
    public func getNv() -> Int32 {
        return _model.pointee.nv
    }
    
    
    public func getNjnt() -> Int32 {
        return _model.pointee.njnt
    }
    
    public func jointType(jointID: Int) -> mjtJoint {
      let rawInt32 = _model.pointee.jnt_type[jointID]
      let rawUInt32 = UInt32(bitPattern: rawInt32)   // reinterpret the bits as UInt32
      return mjtJoint(rawValue: rawUInt32)
        
    }
    

    public func isJointLimited(jointID: Int) -> Bool {
      return _model.pointee.jnt_limited[jointID] != 0
    }
    
    // Qpos address (index into the qpos vector) for this joint
    public func qposAdr(for jointID: Int) -> Int {
      return Int(_model.pointee.jnt_qposadr[jointID])
    }
    
    // Range [min, max] for this joint’s qpos (if limited)
    public func qposRange(jointID: Int) -> (min: Double, max: Double) {
        // jnt_range is an optional pointer to mjtNum
        guard let basePtr = _model.pointee.jnt_range else {
            fatalError("jnt_range is nil on the model")
        }
        let idx = jointID * 2
        // Now we can subscript safely
        let lo = Double(basePtr[idx])
        let hi = Double(basePtr[idx + 1])
        return (min: lo, max: hi)
    }
    
    public func differentiatePos(
        qvel: inout [Double],
        dt: Double,
        qpos1Ptr: UnsafePointer<Double>,
        qpos2Ptr: UnsafePointer<Double>
      ) {
        precondition(qvel.count == Int(_model.pointee.nv))
        qvel.withUnsafeMutableBufferPointer { qPtr in
          mj_differentiatePos(
            _model,
            qPtr.baseAddress!,
            dt,
            qpos1Ptr,
            qpos2Ptr
          )
        }
      }
        
    public func mjJacBody(_ model: MjModel, _ data: MjData, _ jacp: inout [Double], _ jacr: inout [Double], _ bodyId: Int32) {
        precondition(jacp.count == 3 * Int(model._model.pointee.nv))
        precondition(jacr.count == 3 * Int(model._model.pointee.nv))

        jacp.withUnsafeMutableBufferPointer { pPtr in
            jacr.withUnsafeMutableBufferPointer { rPtr in
                mj_jacBody(model._model, data._data, pPtr.baseAddress, rPtr.baseAddress, bodyId)
            }
        }
    }
    
    public func mjJacGeom(_ model: MjModel, _ data: MjData, _ jacp: inout [Double], _ jacr: inout [Double], _ geomId: Int32) {
        precondition(jacp.count == 3 * Int(model._model.pointee.nv))
        precondition(jacr.count == 3 * Int(model._model.pointee.nv))

        jacp.withUnsafeMutableBufferPointer { pPtr in
            jacr.withUnsafeMutableBufferPointer { rPtr in
                mj_jacGeom(model._model, data._data, pPtr.baseAddress, rPtr.baseAddress, geomId)
            }
        }
    }

    
    public func mjJacSite(_ model: MjModel, _ data: MjData, _ jacp: inout [Double], _ jacr: inout [Double], _ siteId: Int32) {
        precondition(jacp.count == 3 * Int(model._model.pointee.nv))
        precondition(jacr.count == 3 * Int(model._model.pointee.nv))

        jacp.withUnsafeMutableBufferPointer { pPtr in
            jacr.withUnsafeMutableBufferPointer { rPtr in
                mj_jacSite(model._model, data._data, pPtr.baseAddress, rPtr.baseAddress, siteId)
            }
        }
    }


}

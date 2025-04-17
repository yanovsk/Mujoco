import C_mujoco

public final class MjData {
    let _data: UnsafeMutablePointer<mjData>
    public let nq: Int32
    public let nv: Int32
    public let na: Int32
    public let nu: Int32
    public let nbody: Int32
    public let nmocap: Int32
    public let nuserdata: Int32
    public let nsensordata: Int32
    
    init(data: UnsafeMutablePointer<mjData>, nq: Int32, nv: Int32, na: Int32, nu: Int32, nbody: Int32, nmocap: Int32, nuserdata: Int32, nsensordata: Int32) {
        _data = data
        self.nq = nq
        self.nv = nv
        self.na = na
        self.nu = nu
        self.nbody = nbody
        self.nmocap = nmocap
        self.nuserdata = nuserdata
        self.nsensordata = nsensordata
    }
    
    deinit {
        mj_deleteData(_data)
    }
    
    // State.
    public var qpos: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.qpos, object: self, len: nq)
        }
        set {
            guard _data.pointee.qpos != newValue._array else { return }
            _data.pointee.qpos.assign(from: newValue._array, count: Int(nq))
        }
    }
    
    
    public var qvel: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.qvel, object: self, len: nv)
        }
        set {
            guard _data.pointee.qvel != newValue._array else { return }
            _data.pointee.qvel.assign(from: newValue._array, count: Int(nv))
        }
    }
    
    var act: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.act, object: self, len: na)
        }
        set {
            guard _data.pointee.act != newValue._array else { return }
            _data.pointee.act.assign(from: newValue._array, count: Int(na))
        }
    }
    
    var qaccWarmstart: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.qacc_warmstart, object: self, len: nv)
        }
        set {
            guard _data.pointee.qacc_warmstart != newValue._array else { return }
            _data.pointee.qacc_warmstart.assign(from: newValue._array, count: Int(nv))
        }
    }
    
    // Control.
    var ctrl: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.ctrl, object: self, len: nu)
        }
        set {
            guard _data.pointee.ctrl != newValue._array else { return }
            _data.pointee.ctrl.assign(from: newValue._array, count: Int(nu))
        }
    }
    
    var qfrcApplied: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.qfrc_applied, object: self, len: nv)
        }
        set {
            guard _data.pointee.qfrc_applied != newValue._array else { return }
            _data.pointee.qfrc_applied.assign(from: newValue._array, count: Int(nv))
        }
    }
    
    var xfrcApplied: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.xfrc_applied, object: self, len: nbody * 6)
        }
        set {
            guard _data.pointee.xfrc_applied != newValue._array else { return }
            _data.pointee.xfrc_applied.assign(from: newValue._array, count: Int(nbody * 6))
        }
    }
    
    // Access mocap_pos (mutable)
        public subscript(mocapPos index: Int) -> [Double] {
            get {
                let ptr = _data.pointee.mocap_pos.advanced(by: 3 * index)
                return [ptr[0], ptr[1], ptr[2]]
            }
            set {
                precondition(newValue.count == 3)
                let ptr = _data.pointee.mocap_pos.advanced(by: 3 * index)
                ptr[0] = newValue[0]
                ptr[1] = newValue[1]
                ptr[2] = newValue[2]
            }
        }

        // Access mocap_quat (mutable)
        public subscript(mocapQuat index: Int) -> [Double] {
            get {
                let ptr = _data.pointee.mocap_quat.advanced(by: 4 * index)
                return [ptr[0], ptr[1], ptr[2], ptr[3]]
            }
            set {
                precondition(newValue.count == 4)
                let ptr = _data.pointee.mocap_quat.advanced(by: 4 * index)
                ptr[0] = newValue[0]
                ptr[1] = newValue[1]
                ptr[2] = newValue[2]
                ptr[3] = newValue[3]
            }
        }
    
    
    // Mocap data.
    public var mocapPos: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.mocap_pos, object: self, len: nmocap * 3)
        }
        set {
            guard _data.pointee.mocap_pos != newValue._array else { return }
            _data.pointee.mocap_pos.assign(from: newValue._array, count: Int(nmocap * 3))
        }
    }
    
    public var mocapQuat: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.mocap_quat, object: self, len: nmocap * 4)
        }
        set {
            guard _data.pointee.mocap_quat != newValue._array else { return }
            _data.pointee.mocap_quat.assign(from: newValue._array, count: Int(nmocap * 4))
        }
    }
    
    // Dynamics.
    var qacc: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.qacc, object: self, len: nv)
        }
        set {
            guard _data.pointee.qacc != newValue._array else { return }
            _data.pointee.qacc.assign(from: newValue._array, count: Int(nv))
        }
    }
    
    var actDot: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.act_dot, object: self, len: na)
        }
        set {
            guard _data.pointee.act_dot != newValue._array else { return }
            _data.pointee.act_dot.assign(from: newValue._array, count: Int(na))
        }
    }
    
    // User data.
    var userdata: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.userdata, object: self, len: nuserdata)
        }
        set {
            guard _data.pointee.userdata != newValue._array else { return }
            _data.pointee.userdata.assign(from: newValue._array, count: Int(nuserdata))
        }
    }
    
    // Sensors.
    var sensordata: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.sensordata, object: self, len: nsensordata)
        }
        set {
            guard _data.pointee.sensordata != newValue._array else { return }
            _data.pointee.sensordata.assign(from: newValue._array, count: Int(nsensordata))
        }
    }
    
    
    // Accessor for body positions: returns a view of the entire xpos array
    public var xpos: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.xpos, object: self, len: nbody * 3)
        }
    }
    
    // Accessor for body orientations (quaternions): returns a view of the entire xquat array
    public var xquat: MjNumArray {
        get {
            MjNumArray(array: _data.pointee.xquat, object: self, len: nbody * 4)
        }
    }
    
    public var geomXpos: MjNumArray {
        // This assumes that _data.pointee.geom_xpos is valid and you know the total number of geom positions (ngeom * 3)
        // You may need to obtain ngeom from your model.
        // For now, assume a placeholder length.
        return MjNumArray(array: _data.pointee.geom_xpos, object: self, len: 0)
    }

    public var siteXpos: MjNumArray {
        // Similarly, placeholder length.
        return MjNumArray(array: _data.pointee.site_xpos, object: self, len: 0)
    }

    public var xmat: MjNumArray {
        return MjNumArray(array: _data.pointee.xmat, object: self, len: Int32(nbody * 9))
    }
    
    public var geomXmat: MjNumArray {
        return MjNumArray(array: _data.pointee.geom_xmat, object: self, len: 0)
    }
    
    public var siteXmat: MjNumArray {
        return MjNumArray(array: _data.pointee.site_xmat, object: self, len: 0)
    }
    
    public func xpos(at id: Int, frameType: String) -> [Double] {
        switch frameType {
        case "body":
            let arr = self.xpos._array
            let offset = id * 3
            return [arr[offset], arr[offset + 1], arr[offset + 2]]
        case "geom":
            let arr = self.geomXpos._array   // you need to provide geomXpos in MjData
            let offset = id * 3
            return [arr[offset], arr[offset + 1], arr[offset + 2]]
        case "site":
            let arr = self.siteXpos._array   // similarly, provide siteXpos
            let offset = id * 3
            return [arr[offset], arr[offset + 1], arr[offset + 2]]
        default:
            fatalError("Unsupported frame type: \(frameType)")
        }
    }
    
    public func xmat(at id: Int, frameType: String) -> [Double] {
        let offset = id * 9
        switch frameType {
        case "body":
            let arr = self.xmat._array
            return (0..<9).map { arr[offset + $0] }
        case "geom":
            let arr = self.geomXmat._array
            return (0..<9).map { arr[offset + $0] }
        case "site":
            let arr = self.siteXmat._array
            return (0..<9).map { arr[offset + $0] }
        default:
            fatalError("Unsupported frame type: \(frameType)")
        }
    }
    
    public func bodyXPos(bodyId: Int) -> [Double] {
        let array = xpos._array
        let offset = bodyId * 3
        return [array[offset], array[offset + 1], array[offset + 2]]
    }

    // Helper: extract quaternion of specific body (4 floats)
    public func bodyXQuat(bodyId: Int) -> [Double] {
        let array = xquat._array
        let offset = bodyId * 4
        return [array[offset], array[offset + 1], array[offset + 2], array[offset + 3]]
    }

}


//TODO: need to move this to some other file. for now if i move it to another file the app doesnt see it

public func mat2quat(_ mat: [Double]) -> [Double] {
    
    print("count", mat.count)
    precondition(mat.count == 9, "mat2quat requires 9 elements for a 3x3 matrix.")

    var quat = [Double](repeating: 0.0, count: 4)
    mju_mat2Quat(&quat, mat)

    return quat
}

public func quat2mat(_ quat: [Double]) -> [Double] {
    precondition(quat.count == 4, "quat2mat requires 4 elements for quaternion [w, x, y, z].")

    var mat = [Double](repeating: 0.0, count: 9)
    mju_quat2Mat(&mat, quat)

    return mat
}


public func mulQuat(_ q1: [Double], _ q2: [Double]) -> [Double] {
    precondition(q1.count == 4 && q2.count == 4, "mulQuat requires 4 elements each")
    var result = [Double](repeating: 0.0, count: 4)
    mju_mulQuat(&result, q1, q2)
    return result
}

import C_mujoco

@inline(__always)
public func mat2Quat(_ xmat: [Double]) -> [Double] {
    precondition(xmat.count == 9)
    var quat = [Double](repeating: 0.0, count: 4)
    var mat = xmat  // Mujoco expects mutable array
    mj_mju_mat2Quat(&quat, &mat)
    return quat
}

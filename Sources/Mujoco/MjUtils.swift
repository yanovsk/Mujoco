//import C_mujoco
//
///// Converts a 3x3 rotation matrix (row-major [Double]) into a quaternion [w, x, y, z].
///// - Parameter mat: Array of 9 Doubles, row-major order (3x3 rotation matrix).
///// - Returns: Array of 4 Doubles [w, x, y, z] (unit quaternion).
//public func mat2quat(_ mat: [Double]) -> [Double] {
//    precondition(mat.count == 9, "mat2quat requires 9 elements for a 3x3 matrix.")
//
//    var quat = [Double](repeating: 0.0, count: 4)
//    mju_mat2Quat(&quat, mat)
//
//    return quat
//}



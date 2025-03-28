// swift-tools-version:5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription


let package = Package(
    name: "Mujoco",
    platforms: [
        .visionOS(.v1) // or iOS, macOS, etc.
    ],
    products: [
        .library(
            name: "Mujoco",
            targets: ["Mujoco"]
        ),
    ],
    targets: [
        // 1) The binary target pointing to your XCFramework
        .binaryTarget(
            name: "C_mujoco",
            path: "XCFrameworks/libMujoco.xcframework"
        ),
        

        .target(
            name: "Mujoco",
            dependencies: ["C_mujoco"],  // depends on the binary
            path: "Sources/Mujoco",
            linkerSettings: [
                .linkedLibrary("c++")
            ]
        )
    ]
)

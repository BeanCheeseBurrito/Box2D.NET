const std = @import("std");
const Build = std.build;
const LazyPath = Build.LazyPath;

pub fn compileLibrary(b: *Build, lib: *Build.Step.Compile) void {
    lib.linkLibC();
    lib.strip = lib.optimize != .Debug;

    if (lib.optimize != .Debug) {
        lib.defineCMacro("NDEBUG", null);
    }

    lib.addIncludePath(LazyPath.relative("../native/box2c/src"));
    lib.addIncludePath(LazyPath.relative("../native/box2c/include"));
    lib.addIncludePath(LazyPath.relative("../native/box2c/extern/simde"));

    lib.addCSourceFiles(&.{
        "../native/box2c/src/aabb.c",
        "../native/box2c/src/allocate.c",
        "../native/box2c/src/array.c",
        "../native/box2c/src/bitset.c",
        "../native/box2c/src/block_array.c",
        "../native/box2c/src/body.c",
        "../native/box2c/src/broad_phase.c",
        "../native/box2c/src/constraint_graph.c",
        "../native/box2c/src/contact.c",
        "../native/box2c/src/contact_solver.c",
        "../native/box2c/src/core.c",
        "../native/box2c/src/distance.c",
        "../native/box2c/src/distance_joint.c",
        "../native/box2c/src/dynamic_tree.c",
        "../native/box2c/src/geometry.c",
        "../native/box2c/src/hull.c",
        "../native/box2c/src/id_pool.c",
        "../native/box2c/src/island.c",
        "../native/box2c/src/joint.c",
        "../native/box2c/src/manifold.c",
        "../native/box2c/src/math_functions.c",
        "../native/box2c/src/motor_joint.c",
        "../native/box2c/src/mouse_joint.c",
        "../native/box2c/src/prismatic_joint.c",
        "../native/box2c/src/revolute_joint.c",
        "../native/box2c/src/shape.c",
        "../native/box2c/src/solver.c",
        "../native/box2c/src/solver_set.c",
        "../native/box2c/src/stack_allocator.c",
        "../native/box2c/src/table.c",
        "../native/box2c/src/timer.c",
        "../native/box2c/src/types.c",
        "../native/box2c/src/weld_joint.c",
        "../native/box2c/src/wheel_joint.c",
        "../native/box2c/src/world.c",
    }, &.{});

    b.installArtifact(lib);
}

pub fn build(b: *Build) void {
    const name = "box2c";
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    compileLibrary(b, b.addSharedLibrary(.{ .name = name, .target = target, .optimize = optimize }));
}

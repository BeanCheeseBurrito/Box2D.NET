const std = @import("std");
const Build = std.Build;
const LazyPath = Build.LazyPath;

pub fn build(b: *Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const lib = b.addSharedLibrary(.{
        .name = "box2d",
        .target = target,
        .optimize = optimize,
        .strip = optimize != .Debug,
    });

    if (optimize != .Debug) {
        lib.defineCMacro("NDEBUG", null);
    }

    lib.linkLibC();

    lib.addIncludePath(b.path("../native/box2d/src"));
    lib.addIncludePath(b.path("../native/box2d/include"));
    lib.addIncludePath(b.path("../native/box2d/extern/simde"));

    lib.addCSourceFiles(.{
        .files = &.{
            "../native/box2d/src/aabb.c",
            "../native/box2d/src/allocate.c",
            "../native/box2d/src/array.c",
            "../native/box2d/src/bitset.c",
            "../native/box2d/src/block_array.c",
            "../native/box2d/src/body.c",
            "../native/box2d/src/broad_phase.c",
            "../native/box2d/src/constraint_graph.c",
            "../native/box2d/src/contact.c",
            "../native/box2d/src/contact_solver.c",
            "../native/box2d/src/core.c",
            "../native/box2d/src/distance.c",
            "../native/box2d/src/distance_joint.c",
            "../native/box2d/src/dynamic_tree.c",
            "../native/box2d/src/geometry.c",
            "../native/box2d/src/hull.c",
            "../native/box2d/src/id_pool.c",
            "../native/box2d/src/island.c",
            "../native/box2d/src/joint.c",
            "../native/box2d/src/manifold.c",
            "../native/box2d/src/math_functions.c",
            "../native/box2d/src/motor_joint.c",
            "../native/box2d/src/mouse_joint.c",
            "../native/box2d/src/prismatic_joint.c",
            "../native/box2d/src/revolute_joint.c",
            "../native/box2d/src/shape.c",
            "../native/box2d/src/solver.c",
            "../native/box2d/src/solver_set.c",
            "../native/box2d/src/stack_allocator.c",
            "../native/box2d/src/table.c",
            "../native/box2d/src/timer.c",
            "../native/box2d/src/types.c",
            "../native/box2d/src/weld_joint.c",
            "../native/box2d/src/wheel_joint.c",
            "../native/box2d/src/world.c",
        },
        .flags = &.{},
    });

    b.installArtifact(lib);
}

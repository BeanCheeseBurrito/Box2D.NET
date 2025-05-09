const std = @import("std");
const builtin = @import("builtin");
const Build = std.Build;

pub const LibraryType = enum { Shared, Static };

const BuildOptions = struct {
    optimize: std.builtin.OptimizeMode,
    target: Build.ResolvedTarget,
    library_type: LibraryType,
    compiler_rt_path: ?[]const u8,
};

const src_flags = [_][]const u8{
    "-fno-sanitize=undefined",
};

const include_paths = [_][]const u8{
    "../native/box2d/src",
    "../native/box2d/include",
    "../native/box2d/extern/simde",
};

const src_files = [_][]const u8{
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
};

pub fn compile(b: *Build, options: BuildOptions) void {
    const lib = switch (options.library_type) {
        .Shared => b.addSharedLibrary(.{
            .name = "box2d",
            .target = options.target,
            .optimize = options.optimize,
            .strip = options.optimize != .Debug,
            .link_libc = true,
        }),
        .Static => b.addStaticLibrary(.{
            .name = "box2d",
            .target = options.target,
            .optimize = options.optimize,
            .strip = options.optimize != .Debug,
            .link_libc = true,
            .root_source_file = if (options.compiler_rt_path) |path| .{ .cwd_relative = path } else null,
        }),
    };

    if (options.optimize != .Debug) {
        lib.defineCMacro("NDEBUG", null);
    }

    for (include_paths) |path| {
        lib.addIncludePath(b.path(path));
    }

    for (src_files) |file| {
        lib.addCSourceFile(.{ .file = b.path(file), .flags = &src_flags });
    }

    switch (options.target.result.os.tag) {
        .windows => {
            // Temporary fix to get rid of undefined symbol errors when statically linking in Native AOT.
            if (options.library_type == LibraryType.Static) {
                // lib.addCSourceFile(.{ .file = b.path("../native/windows.c"), .flags = &src_flags });
            }
        },
        .ios => {
            if (b.sysroot == null) {
                @panic("A --sysroot path to an IOS SDK needs to be provided when compiling for IOS.");
            }

            lib.addSystemFrameworkPath(.{ .cwd_relative = b.pathJoin(&.{ b.sysroot.?, "/System/Library/Frameworks" }) });
            lib.addSystemIncludePath(.{ .cwd_relative = b.pathJoin(&.{ b.sysroot.?, "/usr/include" }) });
            lib.addLibraryPath(.{ .cwd_relative = b.pathJoin(&.{ b.sysroot.?, "/usr/lib" }) });
        },
        .emscripten => {
            if (b.sysroot == null) {
                @panic("Pass '--sysroot \"$EMSDK/upstream/emscripten\"'");
            }

            const cache_include = b.pathJoin(&.{ b.sysroot.?, "cache", "sysroot", "include" });
            var dir = std.fs.openDirAbsolute(cache_include, std.fs.Dir.OpenDirOptions{ .access_sub_paths = true, .no_follow = true }) catch @panic("No emscripten cache. Generate it!");
            dir.close();
            lib.addIncludePath(.{ .cwd_relative = cache_include });
        },
        .linux => {
            if (options.target.result.abi == .android) {
                if (b.sysroot == null) {
                    @panic("A --sysroot path to an Android NDK needs to be provided when compiling for Android.");
                }

                const host_tuple = switch (builtin.target.os.tag) {
                    .linux => "linux-x86_64",
                    .windows => "windows-x86_64",
                    .macos => "darwin-x86_64",
                    else => @panic("unsupported host OS"),
                };

                const triple = switch (options.target.result.cpu.arch) {
                    .aarch64 => "aarch64-linux-android",
                    .x86_64 => "x86_64-linux-android",
                    else => @panic("Unsupported Android architecture"),
                };
                const android_api_level: []const u8 = "21";

                const android_sysroot = b.pathJoin(&.{ b.sysroot.?, "/toolchains/llvm/prebuilt/", host_tuple, "/sysroot" });
                const android_lib_path = b.pathJoin(&.{ android_sysroot, "/usr/lib/", triple, android_api_level });
                const android_include_path = b.pathJoin(&.{ android_sysroot, "/usr/include" });
                const android_system_include_path = b.pathJoin(&.{ android_sysroot, "/usr/include/", triple });

                lib.addLibraryPath(.{ .cwd_relative = android_lib_path });

                const libc_file_name = "android-libc.conf";
                var libc_content = std.ArrayList(u8).init(b.allocator);
                errdefer libc_content.deinit();

                const writer = libc_content.writer();
                const libc_installation = std.zig.LibCInstallation{
                    .include_dir = android_include_path,
                    .sys_include_dir = android_system_include_path,
                    .crt_dir = android_lib_path,
                };
                libc_installation.render(writer) catch @panic("Failed to render libc");
                const libc_path = b.addWriteFiles().add(libc_file_name, libc_content.items);

                lib.setLibCFile(libc_path);
                lib.libc_file.?.addStepDependencies(&lib.step);
            }
        },
        else => {},
    }

    b.installArtifact(lib);
}

pub fn build(b: *Build) void {
    compile(b, .{
        .optimize = b.standardOptimizeOption(.{}),
        .target = b.standardTargetOptions(.{}),
        .library_type = b.option(LibraryType, "library-type", "Compile as a static or shared library.") orelse LibraryType.Shared,
        // When building static libraries for Windows, zig's compiler-rt needs to be bundled.
        // For some reason, setting "bundle_compiler_rt" to true doesn't produce a static library that works with NativeAOT.
        // As a work-around, we manually build the compiler_rt.zig file.
        .compiler_rt_path = b.option([]const u8, "compiler-rt-path", "Path to the compiler_rt file.") orelse null,
    });
}

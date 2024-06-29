using System.Runtime.CompilerServices;
using Bindgen.NET;

const string library = "box2c";

BindingOptions options = new()
{
    Namespace = "Box2D.NET.Bindings",
    Class = "B2",

    DllImportPath = library,
    DllFilePaths =
    {
        library,
        "runtimes/linux-x64/native/" + library,
        "runtimes/linux-arm64/native/" + library,
        "runtimes/osx-x64/native/" + library,
        "runtimes/osx-arm64/native/" + library,
        "runtimes/win-x64/native/" + library,
        "runtimes/win-arm64/native/" + library
    },

    IncludeBuiltInClangHeaders = true,
    IncludeDirectories = { GetNativeDirectory("include") },

    TreatInputFileAsRawSourceCode = true,
    InputFile = """
                #include <box2d/base.h>
                #include <box2d/box2d.h>
                #include <box2d/collision.h>
                #include <box2d/id.h>
                #include <box2d/math_functions.h>
                #include <box2d/types.h>
                """,
    OutputFile = GetOutputDirectory("B2.g.cs"),

    RemappedPrefixes =
    {
        ("b2AABB_", "AABB"),
        ("b2Body_", "Body"),
        ("b2Chain_", "Chain"),
        ("b2DistanceJoint_", "DistanceJoint"),
        ("b2DynamicTree_", "DynamicTree"),
        ("b2Joint_", "Joint"),
        ("b2MotorJoint_", "MotorJoint"),
        ("b2MouseJoint_", "MouseJoint"),
        ("b2PrismaticJoint_", "PrismaticJoint"),
        ("b2RevoluteJoint_", "RevoluteJoint"),
        ("b2Rot_", "Rot"),
        ("b2Shape_", "Shape"),
        ("b2Vec2_", "Vec2"),
        ("b2WheelJoint_", "WheelJoint"),
        ("b2WeldJoint_", "WeldJoint"),
        ("b2World_", "World"),
        ("b2_", ""),
        ("b2", ""),
    },

    GenerateMacros = true,
    GenerateExternVariables = true,
    SuppressedWarnings = { "CS9084" }
};

BindingGenerator.Generate(options);

string GetCurrentFilePath([CallerFilePath] string filePath = "")
{
    return filePath;
}

string GetNativeDirectory(string path)
{
    return GetCurrentFilePath() + "/../../native/box2c/" + path;
}

string GetOutputDirectory(string fileName)
{
    return GetCurrentFilePath() + "/../../Box2D.NET.Bindings/" + fileName;
}

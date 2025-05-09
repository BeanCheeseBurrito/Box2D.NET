using System.Runtime.CompilerServices;
using Bindgen.NET;

BindingOptions options = new()
{
    Namespace = "Box2D.NET.Bindings",
    Class = "B2",

    DllImportPath = "box2d",

    SystemIncludeDirectories = { Path.Combine(BuildConstants.ZigLibPath, "include") },
    IncludeDirectories = { GetIncludeDirectory() },

    TreatInputFileAsRawSourceCode = true,
    OutputFile = GetOutputDirectory(),
    InputFile = """
        #include <box2d/base.h>
        #include <box2d/box2d.h>
        #include <box2d/collision.h>
        #include <box2d/id.h>
        #include <box2d/math_functions.h>
        #include <box2d/types.h>
        """,

    GenerateDisableRuntimeMarshallingAttribute = true,

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
    }
};

BindingGenerator.Generate(options);

string GetCurrentFilePath([CallerFilePath] string filePath = "")
{
    return filePath;
}

string GetIncludeDirectory()
{
    return Path.GetFullPath(GetCurrentFilePath() + "/../../native/box2d/include");
}

string GetOutputDirectory()
{
    return GetCurrentFilePath() + "/../../Box2D.NET.Bindings/B2.g.cs";
}

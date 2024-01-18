# Box2D.NET

<div align="center">

![](https://camo.githubusercontent.com/af9c5a12929eab1361bc603192300b693cf55f6e8fecef77e5d944ae6d0d3bb9/68747470733a2f2f626f7832642e6f72672f696d616765732f6c6f676f2e737667)
  
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg?style=for-the-badge)](https://github.com/BeanCheeseBurrito/Box2D.NET/blob/main/LICENSE)
[![Nuget (with prereleases)](https://img.shields.io/nuget/vpre/Box2D.NET.Release?style=for-the-badge&color=blue)](https://www.nuget.org/packages/Box2D.NET.Release)

</div>

**Box2D.NET** is a C# binding for [Box2C](https://github.com/erincatto/box2c). Low-level bindings to the C api are included and generated with [Bindgen.NET](https://github.com/BeanCheeseBurrito/Bindgen.NET). Native libraries are cross-compiled with [Vezel-Dev's Zig Toolsets](https://github.com/vezel-dev/zig-toolsets).

> [!NOTE]
> There is currently no high-level wrapper but it may come in the future! **Box2D.NET** and **Box2D.NET.Bindings** are identical packages.

## NuGet
You can download the nuget package and use **Box2D.NET** right away!

**Box2D.NET (Wrapper + Bindings + Native Libraries): [Release](https://www.nuget.org/packages/Box2D.NET.Release/) | [Debug](https://www.nuget.org/packages/Box2D.NET.Debug/)**
```console
dotnet add PROJECT package Box2D.NET.Release --version *-*
```

**Box2D.NET.Bindings (Bindings + Native Libraries): [Release](https://www.nuget.org/packages/Box2D.NET.Bindings.Release/) | [Debug](https://www.nuget.org/packages/Box2D.NET.Bindings.Debug/)**
```console
dotnet add PROJECT package Box2D.NET.Bindings.Release --version *-*
```

**Box2D.NET.Native (Native Libraries): [Release](https://www.nuget.org/packages/Box2D.NET.Native.Release/) | [Debug](https://www.nuget.org/packages/Box2D.NET.Native.Debug/)**
```console
dotnet add PROJECT package Box2D.NET.Native.Release --version *-*
```

**Box2D.NET** provides both [release](https://www.nuget.org/packages/Box2D.NET.Release) and [debug](https://www.nuget.org/packages/Box2D.NET.Debug) packages for nuget.
To include both of them in your project based on your build configuration, use the package references below. The latest stable or prerelease versions will be added to your project.
```xml
<Project Sdk="Microsoft.NET.Sdk">

    <PropertyGroup>
        <OutputType>Exe</OutputType>
        <TargetFramework>net7.0</TargetFramework>
    </PropertyGroup>

    <ItemGroup>
        <PackageReference Include="Box2D.NET.Debug" Version="*-*" Condition="'$(Configuration)' == 'Debug'" />
        <PackageReference Include="Box2D.NET.Release" Version="*-*" Condition="'$(Configuration)' == 'Release'" />
    </ItemGroup>

</Project>
```

## GitLab Package Registry
For more up-to-date packages, development builds are available on the [GitLab package registry](https://gitlab.com/BeanCheeseBurrito/Box2D.NET/-/packages). To add the development feed to your project, add the GitLab link below  as a restore source. You can now reference any package version listed [here](https://gitlab.com/BeanCheeseBurrito/Box2D.NET/-/packages)!
```xml
<Project Sdk="Microsoft.NET.Sdk">

    <PropertyGroup>
        <OutputType>Exe</OutputType>
        <TargetFramework>net7.0</TargetFramework>
        <RestoreAdditionalProjectSources>https://gitlab.com/api/v4/projects/53993416/packages/nuget/index.json</RestoreAdditionalProjectSources>
    </PropertyGroup>

    <ItemGroup>
        <PackageReference Include="Box2D.NET.Debug" Version="0.0.4"/>
    </ItemGroup>

</Project>
```
> [!WARNING] 
> Development feed packages may be deleted without warning to free up space.

## Running examples
To run any of the example programs, use ``dotnet run``and set the `Example` property's value to the example's full class name.
**Example**:
```console
dotnet run --project Box2D.NET.Examples --property:Example=HelloWorld
```

## Building from source
### Clone the repo
Clone the repo and it's submodules.
```console
git clone --recursive https://github.com/BeanCheeseBurrito/Box2D.NET.git
cd Box2D.NET
```
### Restore dependencies
Run the following command on the solution to restore all project dependencies.
```console
dotnet restore
```
### Generate bindings
Generate the binding code. Bindings are generated with [Bindgen.NET](https://github.com/BeanCheeseBurrito/Bindgen.NET).
```console
dotnet run --project Box2D.NET.Bindgen
```
### Build Box2D.NET
Compile the bindings and native libraries. The [zig](https://ziglang.org/learn/overview/#cross-compiling-is-a-first-class-use-case) compiler will automatically be downloaded and cached in your local nuget package folder. Native libraries will be cross-compiled for linux, macos, and windows.
```console
dotnet build
```

### Reference the project
Reference the project and import the native libraries.

```xml
<Project Sdk="Microsoft.NET.Sdk">

    <PropertyGroup>
        <OutputType>Exe</OutputType>
        <TargetFramework>net7.0</TargetFramework>
    </PropertyGroup>

    <Import Project="PATH/Box2D.NET/Box2D.NET.Native/Box2D.NET.Native.targets" />

    <ItemGroup>
        <ProjectReference Include="PATH/Box2D.NET/Box2D.NET/Box2D.NET.csproj" />
    </ItemGroup>

</Project>
```

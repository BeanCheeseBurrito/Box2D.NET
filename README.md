# Box2D.NET

<div align="center">

![](https://camo.githubusercontent.com/af9c5a12929eab1361bc603192300b693cf55f6e8fecef77e5d944ae6d0d3bb9/68747470733a2f2f626f7832642e6f72672f696d616765732f6c6f676f2e737667)
  
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg?style=for-the-badge)](https://github.com/BeanCheeseBurrito/Box2D.NET/blob/main/LICENSE)
[![Nuget (with prereleases)](https://img.shields.io/nuget/vpre/Box2D.NET.Release?style=for-the-badge&color=blue)](https://www.nuget.org/packages/Box2D.NET.Release)

[Examples](https://github.com/BeanCheeseBurrito/Box2D.NET/tree/main/Box2D.NET.Examples/CSharp)

</div>

**Box2D.NET** is a C# binding for [Box2D](https://github.com/erincatto/box2d). Low-level bindings to the C api are included and generated with [Bindgen.NET](https://github.com/BeanCheeseBurrito/Bindgen.NET). Native libraries are cross-compiled with [Vezel-Dev's Zig Toolsets](https://github.com/vezel-dev/zig-toolsets).

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
        <TargetFramework>net8.0</TargetFramework>
    </PropertyGroup>

    <ItemGroup>
        <PackageReference Include="Box2D.NET.Debug" Version="*-*" Condition="'$(Configuration)' == 'Debug'" />
        <PackageReference Include="Box2D.NET.Release" Version="*-*" Condition="'$(Configuration)' == 'Release'" />
    </ItemGroup>

</Project>
```

## GitHub Package Registry
For more up-to-date packages, development builds are available on the [GitHub package registry](https://github.com/BeanCheeseBurrito?tab=packages&repo_name=Box2D.NET). Packages are automatically uploaded on every commit to the main branch.

To access development builds from your project, you first need to create a GitHub personal access token with the ``read:packages`` permission. (See [Creating a personal access token (classic)](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-personal-access-token-classic))

Once you have created a personal access token, run the following command to add the GitHub feed as a new package source. Replace ``YOUR_GITHUB_USERNAME`` with your GitHub username and ``YOUR_GITHUB_TOKEN`` with your personal access token.
```bash
dotnet nuget add source --name "box2d.net" --username "YOUR_GITHUB_USERNAME" --password "YOUR_GITHUB_TOKEN" --store-password-in-clear-text "https://nuget.pkg.github.com/BeanCheeseBurrito/index.json"
```

You can now reference any package from the [GitHub feed](https://github.com/BeanCheeseBurrito?tab=packages&repo_name=Box2D.NET)!

```console
dotnet add PROJECT package Box2D.NET.Release --version *-build.*
```

```xml
<Project Sdk="Microsoft.NET.Sdk">
    <PropertyGroup>
        <OutputType>Exe</OutputType>
        <TargetFramework>net8.0</TargetFramework>
    </PropertyGroup>

    <ItemGroup>
        <PackageReference Include="Box2D.NET.Debug" Version="*-build.*"/>
    </ItemGroup>
</Project>
```
___
By default, the GitHub feed will be added to your global ``nuget.config`` file and can be referenced by any project on your machine. If wish to add the feed to a single project/solution, create a ``nuget.config`` file at the root of your project/solution directory and run the following command with the ``--configfile`` option.
```bash
dotnet nuget add source --configfile "./nuget.config" --name "box2d.net" --username "YOUR_GITHUB_USERNAME" --password "YOUR_GITHUB_TOKEN" --store-password-in-clear-text "https://nuget.pkg.github.com/BeanCheeseBurrito/index.json"
```
To remove the GitHub feed from your NuGet package sources, run the following command.
```bash
dotnet nuget remove source "box2d.net"
```
GitHub Actions workflows can be authenticated using the ``GITHUB_TOKEN`` secret.
```yaml
- name: Add GitHub source
  run: dotnet nuget add source --name "box2d.net" --username "USERNAME" --password "${{ secrets.GITHUB_TOKEN }}" --store-password-in-clear-text "https://nuget.pkg.github.com/BeanCheeseBurrito/index.json"
```
> [!WARNING]
> Development feed packages may be deleted without warning to free up space.

## Running examples
To run any of the example programs, use ``dotnet run`` and set the `Example` property's value to the example's full class name.
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
        <TargetFramework>net8.0</TargetFramework>
    </PropertyGroup>

    <ItemGroup>
        <ProjectReference Include="PATH/Box2D.NET/Box2D.NET/Box2D.NET.csproj"/>
    </ItemGroup>

</Project>
```

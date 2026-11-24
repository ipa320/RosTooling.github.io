# Building, testing and releasing

This page is for people who work on RosTooling itself. There are two repositories: [RosTooling](https://github.com/ipa320/RosTooling), the core with the languages, generators and servers, and [RosTooling_Extension](https://github.com/ipa-esa/RosTooling_Extension), the VS Code extension that bundles a core.

## Core (RosTooling)

### Build and test

The core is a Maven and Tycho build. You need Java 21 and use the Maven wrapper of the repository:

```bash
./mvnw clean verify -f plugins/de.fraunhofer.ipa.ros.parent/pom.xml
```

The same command runs in the `build` workflow on every push and pull request. It includes the tests of the languages and the generators.

### Golden files

The generators are tested against golden files: for every supported distribution and for C++ and Python, the generated package is compared with a stored copy. When you change a template on purpose, update the copies with:

```bash
ROSTOOLING_UPDATE_GOLDEN=1 ./mvnw verify -f plugins/de.fraunhofer.ipa.ros.parent/pom.xml
```

and review the diff of the golden files like any other change.

### Distribution matrix

Golden files only show that the output did not change. The `distro-matrix` workflow shows that it works: it builds the golden packages inside the `ros:<distro>` image of each supported distribution, treats deprecations as errors, starts every executable and launches the system packages. It runs on demand and publishes one status badge per distribution. The script behind it is `tools/distro-matrix/build-golden.sh`, which you can run locally in the same containers.

### Compatibility check

`tools/compat` compares the models and the generated output of a branch with those of the last release (`snapshot.sh`, then `compare.sh`). Models that parse in a release must keep parsing, see the design rules in its README.

### Releasing

1. Update `CHANGELOG.md` and set the new version with `mvn versions:set -DnewVersion=X.Y.Z -f plugins/de.fraunhofer.ipa.ros.parent/pom.xml`.
2. Create a release on GitHub. The `maven-publish` workflow publishes the bundles to GitHub Packages.
3. A push to the `devel` branch publishes `X.(Y+1).0-SNAPSHOT` builds, which the extension can use before a release exists.

## VS Code extension (RosTooling_Extension)

### Build and test

```bash
./gradlew installExtension
npm test
```

`installExtension` builds the language server JAR against the core version in `build.gradle`, bundles the Studio webview, packages the extension and installs it. `npm test` runs the unit and integration tests, which start the real language server and GLSP server. The `ci` workflow does the same on every push.

### Releasing

1. Update `CHANGELOG.md` and the version in `package.json`, and set the core version in `build.gradle`.
2. Publish a release on GitHub. The `gradle-publish` workflow builds the extension and publishes it to the Open VSX Registry. The VSIX file is attached to the release.
3. The same workflow can be started by hand, with these inputs: whether to publish to Open VSX, whether to publish the MCP server as the npm package `rostooling-mcp`, whether it is a pre-release, and which RosTooling version or branch (`rostooling_version`, `rostooling_ref`) and Studio branch (`studio_ref`) to build against. A pre-release uses the newest `devel` SNAPSHOT of the core by default.

## Eclipse application (legacy)

The Eclipse feature is built from the same core sources and is exported from Eclipse as *Deployable features*, then copied to the update site. The update site holds an old release, because the Eclipse application is not released with the other parts any more, see [Installation](Installation.md#eclipse-legacy). To run a plugin inside Eclipse from source, see [Running a plugin inside Eclipse](EclipsePlugin.md).

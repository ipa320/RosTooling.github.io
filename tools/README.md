# Tools

## check_examples.py

Checks the model examples in the documentation with the real RosTooling language server, so the pages cannot show syntax that the tooling rejects.

An example is a fenced code block with an HTML comment above it:

    <!-- check: group=basic file=demo.rossystem -->

Blocks with the same `group` are written to one folder and validated together (a `.rossystem` example needs the `.ros2` examples it refers to). A block must produce no error and no warning. Add `skip="<reason>"` to a block whose feature the shipped server does not accept yet; it is listed and not run, and the attribute is removed once the server is fixed. Add `expect="<text>"` to a block that shows a diagnostic on purpose; the check then requires a diagnostic that contains the text.

```bash
python3 tools/check_examples.py --server /path/to/ipa-rostooling-studio/scripts/ros_server.py
```

The server script is the one from the `ipa-rostooling-studio` repository. It needs Java 21 and finds a RosTooling language server JAR (for example the one built into the VS Code extension). `ROSTOOLING_SERVER_SCRIPT` can be set instead of `--server`. `--list` prints the examples that were found without running them.

`--include-skipped` runs the skipped groups too. Use it with a server that has the fix; the language server JAR can be chosen with the environment variable `ROSMODEL_JAR`.

`plugin=<id>` marks a group that needs a RosTooling plugin. It runs only when the id is in `ROS_PLUGINS_ENABLED` and `ROS_PLUGINS_DIR` points to the approved JAR, for example the BehaviorTree plugin. `with=<folder>` adds the files of a folder of this repository to the group; `tools/fixtures/bt/` holds copies of the catalogue models of the BehaviorTree plugin.

Not every language file can be checked this way: the server script reads `.ros`, `.ros2` and `.rossystem` files, so `.ros1` examples are not covered.

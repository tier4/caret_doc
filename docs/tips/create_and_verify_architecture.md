# Creation and verification of architecture files with CLI

CARET provides CLI to create architecture files and verify paths.

## Creation of architecture file

The following commands can be used to create an architecture file that has minimum descriptions from trace data.

```bash
ros2 caret create_architecture_file [PATH_TO_CTF] -o [OUTPUT_PATH]
```

After creating the architecture file, please refer to [How to define a target path](https://tier4.github.io/CARET_doc/latest/tutorials/configuration/#how-to-define-a-target-path).

## Verification of paths

The following command executes the `path.verify()` method on paths defined in the architecture file.

```bash
ros2 caret verify_paths [PATH_TO_ARCHITECTURE_FILE] -p [DEFINED_PATH_NAMES]
```

- `-p` is optional; by default, `path.verify()` is executed for all defined paths.
- Any number of [DEFINED_PATH_NAMES] can be inputted.

Please refer to [How to define latency of a single node](https://tier4.github.io/CARET_doc/latest/tutorials/configuration/#how-to-define-latency-of-a-single-node) for details on the `path.verify()` method and how to handle output warnings.

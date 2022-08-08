# Creation and verification of architecture files with CLI

CARET provides CLI to create architecture files and verify paths.

## Creation of architecture file

The architecture file describes the application structure and latency definitions.
The following commands can create an architecture file template including only the application structure section.

```bash
ros2 caret create_architecture_file [PATH_TO_CTF] -o [OUTPUT_PATH]
```

After creating the architecture file, please refer to [How to define a target path](https://tier4.github.io/CARET_doc/latest/tutorials/configuration/#how-to-define-a-target-path).

## Verification of paths

The following command verifies whether latency definitions are set sufficiently.

```bash
ros2 caret verify_paths [PATH_TO_ARCHITECTURE_FILE] -p [DEFINED_PATH_NAMES]
```

- `-p` is optional; by default, all defined paths are verified.
- Any number of [DEFINED_PATH_NAMES] can be inputted.

This command executes the `path.verify()` method.
Please refer to [How to define latency of a single node](https://tier4.github.io/CARET_doc/latest/tutorials/configuration/#how-to-define-latency-of-a-single-node) for details on the `path.verify()` method and how to handle output warnings.

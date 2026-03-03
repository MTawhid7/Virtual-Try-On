import os
import re

src_dir = "crates/vistio-contact/src"
tests_dir = os.path.join(src_dir, "tests")
os.makedirs(tests_dir, exist_ok=True)

test_files = [
    "barrier.rs",
    "bvh.rs",
    "ccd.rs",
    "distance_primitives.rs",
    "ipc_response.rs"
]

mod_rs_content = ""

for file_name in test_files:
    file_path = os.path.join(src_dir, file_name)
    with open(file_path, "r") as f:
        content = f.read()

    match = re.search(r"#\[cfg\(test\)\]\s*mod\s+tests\s*\{", content)
    if not match:
        continue

    start_idx = match.start()

    # Extract test content
    test_content = content[start_idx:]

    # Remove from original
    new_content = content[:start_idx]
    with open(file_path, "w") as f:
        f.write(new_content)

    # Write to new test file
    test_file_name = file_name.replace(".rs", "_tests.rs")

    inner_match = re.search(r"#\[cfg\(test\)\]\s*mod\s+tests\s*\{([\s\S]*)\}\s*$", test_content)
    if inner_match:
        inner = inner_match.group(1)
        module_name = file_name.replace(".rs", "")
        # Remove use super::*; and add use crate::module_name::*;
        inner = re.sub(r"use\s+super\s*::\s*\*;", f"use crate::{module_name}::*;", inner)

        test_path = os.path.join(tests_dir, test_file_name)
        with open(test_path, "w") as f:
            f.write(inner.strip() + "\n")

        mod_name = test_file_name.replace(".rs", "")
        mod_rs_content += f"mod {mod_name};\n"

with open(os.path.join(tests_dir, "mod.rs"), "w") as f:
    f.write(mod_rs_content)
print("Extraction complete")

import re
import os

src_dir = "crates/vistio-contact/src"
files = ["barrier.rs", "bvh.rs", "ccd.rs", "distance_primitives.rs", "ipc_response.rs"]

for file in files:
    path = os.path.join(src_dir, file)
    with open(path, "r") as f:
        content = f.read()
    
    # Make top-level functions pub(crate) if they are just fn
    content = re.sub(r"^fn\s+([a-zA-Z0-9_]+)", r"pub(crate) fn \1", content, flags=re.MULTILINE)
    # Make structs pub(crate)
    content = re.sub(r"^struct\s+([a-zA-Z0-9_]+)", r"pub(crate) struct \1", content, flags=re.MULTILINE)
    # Make enums pub(crate)
    content = re.sub(r"^enum\s+([a-zA-Z0-9_]+)", r"pub(crate) enum \1", content, flags=re.MULTILINE)
    
    # Also add pub(crate) to constants
    content = re.sub(r"^const\s+([a-zA-Z0-9_]+)", r"pub(crate) const \1", content, flags=re.MULTILINE)

    with open(path, "w") as f:
        f.write(content)

test_dir = os.path.join(src_dir, "tests")
test_files = [f for f in os.listdir(test_dir) if f.endswith(".rs")]
for file in test_files:
    path = os.path.join(test_dir, file)
    with open(path, "r") as f:
        content = f.read()
    
    # Add common imports to the top
    imports = "use vistio_math::Vec3;\nuse crate::broad::BroadPhase;\n"
    if imports not in content:
        content = imports + content
        
    with open(path, "w") as f:
        f.write(content)

print("Visibility fixed")

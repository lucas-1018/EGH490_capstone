# Check Code Format (lint) locally
This assumes that you are using LINUX system
To Run this script, you need to have the following system packages installed:
```bash
sudo apt-get install -y python3 clang-format libxml2-utils shellcheck
python3 -m pip install black flake8 cmakelang
```

Also, need to make this script executable
```bash
chmod +x lint_local.sh
```

To run the script, execute the following command:
```bash
# From dave root directory run lint_local.sh
./extras/lint_local/lint_local.sh
```

#!/bin/bash

# To Run this script, you need to have the following system packages installed:
# sudo apt-get install -y python3 clang-format libxml2-utils shellcheck
# python3 -m pip install black flake8 cmakelang

# Also, need to make this script executable
# chmod +x lint_local.sh

# This assumes that you are using LINUX system

# Function to check system packages
check_system_package() {
  dpkg -s "$1" &>/dev/null
  # shellcheck disable=SC2181
  if [ $? -ne 0 ]; then
    echo -e "\033[31mMissing system package: $1\033[0m"
    echo -e "Please install it using:\033[33m sudo apt-get install -y $1\033[0m"
    all_dependencies_installed=false
  fi
}

# Function to check python packages
check_python_package() {
  python3 -m pip show "$1" &>/dev/null
  # shellcheck disable=SC2181
  if [ $? -ne 0 ]; then
    echo -e "\033[31mMissing Python package: $1\033[0m"
    echo -e "Please install it using: \033[33m python3 -m pip install $1\033[0m"
    all_dependencies_installed=false
  fi
}

echo "Checking system dependencies to run lints..."
all_dependencies_installed=true

# List of system packages to check
system_packages=("python3" "clang-format" "libxml2-utils" "shellcheck")

for package in "${system_packages[@]}"
do
  check_system_package "$package"
done

# Check if Python 3 is installed before checking Python packages
if ! command -v python3 &>/dev/null; then
  echo -e "\033[31mPython is not installed. Install to proceed.\033[0m"
else
  echo "Checking Python dependencies to run lints..."

  # List of Python packages to check
  python_packages=("black" "flake8" "cmakelang")

  for pypackage in "${python_packages[@]}"
  do
    check_python_package "$pypackage"
  done
fi

if [ "$all_dependencies_installed" = true ]; then
    echo "Checking out the code for code format lint..."

    echo -e "\033[32m[1/6] =========  Running Python Linters   ========\033[0m"
    black . --check --line-length 100
    flake8 . --max-line-length 100

    echo -e "\033[32m[2/6] =========  Running C++ Linter    ===========\033[0m"
    # shellcheck disable=SC2038
    find . -iname '*.cpp' -o -iname '*.h' | xargs clang-format -i

    echo -e "\033[32m[3/6] =========  Linting XML files     ===========\033[0m"
    find . \( -iname '*.urdf' -o -iname '*.sdf' -o -iname '*.xacro' \
      -o -iname '*.xml' -o -iname '*.launch' \) -print0 | xargs -0 xmllint --noout

    echo -e "\033[32m[4/6] =========  Running ShellCheck    ===========\033[0m"
    # shellcheck disable=SC2038
    find . -iname '*.sh' | while read -r file; do
      awk '{ if (length($0) > 100) \
      printf "\033[31mError in file %s: Line %d is %d characters long \
      (should be under 100):\n%s\n\033[0m", \
      FILENAME, NR, length($0), $0 }' "$file"
    done

    echo -e "\033[32m[5/6] =========  Linting YAML files    ===========\033[0m"
    find . \( -iname '*.yml' -o -iname '*.yaml' \) -print0 | xargs -0 yamllint

    echo -e "\033[32m[6/6] =========  Linting CMake files   ===========\033[0m"
    find . \( -iname 'CMakeLists.txt' -o -iname '*.cmake' \) -print0 | xargs -0 cmake-lint

    # shellcheck disable=SC2028
    echo -e "\033[32mAll linting completed. Awesome! Now, ready to make PR. 🤘🚀 \033[0m"
else
    echo -e "\033[31mNot all dependencies are installed. \
    Please install the missing packages before proceeding.\033[0m"
fi

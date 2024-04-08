#!/bin/bash

# Format the code using clang-format

# Check if clang-format is installed
if ! [ -x "$(command -v clang-format)" ]; then
  echo 'Error: clang-format is not installed.' >&2
  exit 1
fi

# Find all C, C++ and header files excluding specified directories
FILES=$(find . \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
                -not \( -path "./build/*" -o -path "./MicrasFirmware/*" \))

# Loop through each file and format if found
for FILE in $FILES; do
    # Check if the file exists and is readable
    if [ -f "$FILE" ] && [ -r "$FILE" ]; then
        # Format the file using clang-format
        clang-format -style=file -i "$FILE"
        echo "Formatted: $FILE"
    fi
done

echo "Formatting complete."

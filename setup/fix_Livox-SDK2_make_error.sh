DEFINE_H="$HOME/ws_livox/src/Livox-SDK2/sdk_core/comm/define.h"
FILE_MANAGER_H="$HOME/ws_livox/src/Livox-SDK2/sdk_core/logger_handler/file_manager.h"
add_include_once_after_block() {
  local file="$1"

  # If already present, do nothing
  if grep -Eq '^[[:space:]]*#include[[:space:]]*<cstdint>[[:space:]]*$' "$file"; then
    exit 0
  fi

  awk '
    BEGIN { have=0; added=0; inblock=0 }
    {
      if ($0 ~ /^[[:space:]]*#include[[:space:]]*<cstdint>[[:space:]]*$/) { have=1 }
      if ($0 ~ /^[[:space:]]*#include[[:space:]]*[<"].*[>"][[:space:]]*$/) {
        inblock=1
        print
        next
      }
      if (inblock && !have && !added) {
        print "#include <cstdint>"
        added=1
      }
      inblock=0
      print
    }
  ' "$file" > "$file.tmp" && mv "$file.tmp" "$file"
}
add_include_once_after_block "$DEFINE_H"
add_include_once_after_block "$FILE_MANAGER_H"
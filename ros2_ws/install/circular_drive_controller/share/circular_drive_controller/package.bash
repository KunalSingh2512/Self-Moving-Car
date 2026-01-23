# generated from colcon_bash/shell/template/package.bash.em

if [ -z "$COLCON_CURRENT_PREFIX" ]; then
  _colcon_package_bash_COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`/../.." > /dev/null && pwd)"
else
  _colcon_package_bash_COLCON_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
fi

_colcon_package_bash_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \"$1\""
    fi
    . "$@"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}


_colcon_package_bash_source_script "$_colcon_package_bash_COLCON_CURRENT_PREFIX/share/circular_drive_controller/package.sh"

unset _colcon_package_bash_source_script
unset _colcon_package_bash_COLCON_CURRENT_PREFIX

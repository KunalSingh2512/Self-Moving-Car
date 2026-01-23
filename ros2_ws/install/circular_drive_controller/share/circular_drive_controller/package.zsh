# generated from colcon_zsh/shell/template/package.zsh.em

if [ -z "$COLCON_CURRENT_PREFIX" ]; then
  # the prefix is two levels up from the package specific share directory
  _colcon_package_zsh_COLCON_CURRENT_PREFIX="$(builtin cd -q "`dirname "${(%):-%N}"`/../.." > /dev/null && pwd)"
else
  _colcon_package_zsh_COLCON_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
fi

_colcon_package_zsh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \"$1\""
    fi
    . "$@"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

colcon_zsh_convert_to_array() {
  local _listname=$1
  local _dollar="$"
  local _split="{="
  local _to_array="(\"$_dollar$_split$_listname}\")"
  eval $_listname=$_to_array
}

_colcon_package_zsh_source_script "$_colcon_package_zsh_COLCON_CURRENT_PREFIX/share/circular_drive_controller/package.sh"
unset convert_zsh_to_array

unset _colcon_package_zsh_source_script
unset _colcon_package_zsh_COLCON_CURRENT_PREFIX

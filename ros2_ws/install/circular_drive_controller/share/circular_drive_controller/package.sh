# generated from colcon_core/shell/template/package.sh.em

_colcon_prepend_unique_value() {
  _listname="$1"
  _value="$2"

  eval _values=\"\$$_listname\"
  _colcon_prepend_unique_value_IFS=$IFS
  IFS=":"
  _all_values="$_value"
  if [ "$(command -v colcon_zsh_convert_to_array)" ]; then
    colcon_zsh_convert_to_array _values
  fi
  for _item in $_values; do
    if [ -z "$_item" ]; then
      continue
    fi
    if [ "$_item" = "$_value" ]; then
      continue
    fi
    _all_values="$_all_values:$_item"
  done
  unset _item
  IFS=$_colcon_prepend_unique_value_IFS
  unset _colcon_prepend_unique_value_IFS
  eval export $_listname=\"$_all_values\"
  unset _all_values
  unset _values

  unset _value
  unset _listname

_colcon_package_sh_COLCON_CURRENT_PREFIX="/home/kunal-singh/Desktop/Self_Moving_Car/ros2_ws/install/circular_drive_controller"
if [ -z "$COLCON_CURRENT_PREFIX" ]; then
  if [ ! -d "$_colcon_package_sh_COLCON_CURRENT_PREFIX" ]; then
    echo "The build time path \"$_colcon_package_sh_COLCON_CURRENT_PREFIX\" doesn't exist. Either source a script for a different shell or set the environment variable \"COLCON_CURRENT_PREFIX\" explicitly." 1>&2
    unset _colcon_package_sh_COLCON_CURRENT_PREFIX
    return 1
  fi
  COLCON_CURRENT_PREFIX="$_colcon_package_sh_COLCON_CURRENT_PREFIX"
fi
unset _colcon_package_sh_COLCON_CURRENT_PREFIX

_colcon_package_sh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \"$1\""
    fi
    . "$@"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

_colcon_package_sh_source_script "$COLCON_CURRENT_PREFIX/share/circular_drive_controller/hook/pythonpath.sh"
_colcon_package_sh_source_script "$COLCON_CURRENT_PREFIX/share/circular_drive_controller/hook/ament_prefix_path.sh"

unset _colcon_package_sh_source_script
unset COLCON_CURRENT_PREFIX
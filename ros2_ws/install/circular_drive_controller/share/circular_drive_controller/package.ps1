# generated from colcon_powershell/shell/template/package.ps1.em


function colcon_append_unique_value {
  param (
    $_listname,
    $_value
  )

  if (Test-Path Env:$_listname) {
    $_values=(Get-Item env:$_listname).Value
  } else {
    $_values=""
  }
  $_duplicate=""
  $_all_values=""
  if ($_values) {
    $_values.Split(";") | ForEach {
      if ($_) {
        if ($_ -eq $_value) {
          $_duplicate="1"
        }
        if ($_all_values) {
          $_all_values="${_all_values};$_"
        } else {
          $_all_values="$_"
        }
      }
    }
  }
  if (!$_duplicate) {
    if ($_all_values) {
      $_all_values="${_all_values};${_value}"
    } else {
      $_all_values="${_value}"
    }
  }

  Set-Item env:\$_listname -Value "$_all_values"
}

function colcon_prepend_unique_value {
  param (
    $_listname,
    $_value
  )

  if (Test-Path Env:$_listname) {
    $_values=(Get-Item env:$_listname).Value
  } else {
    $_values=""
  }
  $_all_values="$_value"
  if ($_values) {
    $_values.Split(";") | ForEach {
      if ($_) {
        if ($_ -ne $_value) {
          $_all_values="${_all_values};$_"
        }
      }
    }
  }
  Set-Item env:\$_listname -Value "$_all_values"
}

function colcon_package_source_powershell_script {
  param (
    $_colcon_package_source_powershell_script
  )
  if (Test-Path $_colcon_package_source_powershell_script) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_package_source_powershell_script'"
    }
    . "$_colcon_package_source_powershell_script"
  } else {
    Write-Error "not found: '$_colcon_package_source_powershell_script'"
  }
}


$env:COLCON_CURRENT_PREFIX=(Get-Item $PSCommandPath).Directory.Parent.Parent.FullName

colcon_package_source_powershell_script "$env:COLCON_CURRENT_PREFIX\share/circular_drive_controller/hook/pythonpath.ps1"
colcon_package_source_powershell_script "$env:COLCON_CURRENT_PREFIX\share/circular_drive_controller/hook/ament_prefix_path.ps1"

Remove-Item Env:\COLCON_CURRENT_PREFIX

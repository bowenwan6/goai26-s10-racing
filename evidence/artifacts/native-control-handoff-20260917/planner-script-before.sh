#!/bin/bash
node_type=${1:-drdds}

SCRIPT_PATH=$(dirname "$(readlink -f "$0")")
echo "脚本路径: "
echo $SCRIPT_PATH

# Ubuntu 22.04/24.04：postinst 以 user 运行且无 sudo；20.04 及其它保持 sudo
SUDO="sudo "
if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  if [ "${ID:-}" = "ubuntu" ]; then
    case "${VERSION_ID}" in
      22.04|24.04)
        SUDO=""
        ;;
      20.04)
        ;;
      *)
        ;;
    esac
  fi
fi

# 接可通行域
${SUDO}chrt 50 taskset -c 4,5 "$SCRIPT_PATH/../bin/pcl_remove" &

# 接普通点云     确认话题 /LOC_BODY_POINTS 是否有效存在
#${SUDO}chrt 50 taskset -c 4,5 "$SCRIPT_PATH/../bin/pcl_pass_grid" &

${SUDO}chrt 50 taskset -c 4,5 "$SCRIPT_PATH/../bin/localPlanner"

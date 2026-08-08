#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  convert_sequences.sh --dataset DATASET --source-root RAW_ROOT --output CONVERTED_ROOT [options] [SEQUENCE ...]

Required:
  --dataset DATASET          mulran, helipr, semantic_kitti
  --source-root RAW_ROOT     Directory containing sequence folders
  --output CONVERTED_ROOT    Converted dataset root

Options:
  --overwrite                Recreate existing converted sequences
  --continue-on-error        Continue with next sequence after a failure
  --delete-source-after-success
                             Delete each raw sequence only after a verified copy conversion
  --start-lidar-frame N      Pass through to dataloader_convert.py
  --end-lidar-frame N        Pass through to dataloader_convert.py
  --link-mode MODE           copy, reference, symlink, hardlink, hardlink_or_copy
  -h, --help                 Show this help

Examples:
  rosrun dataloader convert_sequences.sh \
    --dataset helipr \
    --source-root /data/raw/helipr \
    --output /data/converted

  rosrun dataloader convert_sequences.sh \
    --dataset semantic_kitti \
    --source-root /data/raw/semantic_kitti \
    --output /data/converted \
    00 01 02 05 07
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

dataset=""
source_root=""
output_root=""
overwrite=0
continue_on_error=0
delete_source_after_success=0
link_mode=""
start_lidar_frame=""
end_lidar_frame=""
sequences=()

validate_converted_copy() {
  local output_dir="$1"
  python3 - "$output_dir" <<'PY'
import sys

from dataloader import validate_dataset

report = validate_dataset(sys.argv[1])

if not report.ok:
    for issue in report.errors:
        print("VALIDATION FAILED [{}] {}".format(issue.code, issue.message), file=sys.stderr)
    raise SystemExit(1)

if report.stats.get("storage_mode") != "copy":
    print(
        "VALIDATION FAILED: storage_mode is '{}', expected 'copy'".format(
            report.stats.get("storage_mode")
        ),
        file=sys.stderr,
    )
    raise SystemExit(1)

print(
    "VALIDATION OK: events={}, unique_files={}".format(
        report.stats["events"],
        report.stats["unique_files"],
    )
)
PY
}

delete_source_sequence() {
  local source_root_real source_dir_real
  source_root_real="$(realpath "$source_root")"
  source_dir_real="$(realpath "$1")"

  if [[ "$source_dir_real" == "/" || "$source_dir_real" == "$source_root_real" ]]; then
    echo "DELETE REFUSED: unsafe source path: $source_dir_real" >&2
    return 1
  fi
  case "$source_dir_real" in
    "$source_root_real"/*) ;;
    *)
      echo "DELETE REFUSED: source is not under source root: $source_dir_real" >&2
      return 1
      ;;
  esac

  echo "DELETE SOURCE: $source_dir_real"
  rm -rf -- "$source_dir_real"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset)
      dataset="${2:-}"
      shift 2
      ;;
    --source-root)
      source_root="${2:-}"
      shift 2
      ;;
    --output)
      output_root="${2:-}"
      shift 2
      ;;
    --overwrite)
      overwrite=1
      shift
      ;;
    --continue-on-error)
      continue_on_error=1
      shift
      ;;
    --delete-source-after-success)
      delete_source_after_success=1
      shift
      ;;
    --link-mode)
      link_mode="${2:-}"
      shift 2
      ;;
    --start-lidar-frame)
      start_lidar_frame="${2:-}"
      shift 2
      ;;
    --end-lidar-frame)
      end_lidar_frame="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      die "unknown option: $1"
      ;;
    *)
      sequences+=("$1")
      shift
      ;;
  esac
done

[[ -n "$dataset" ]] || die "--dataset is required"
[[ -n "$source_root" ]] || die "--source-root is required"
[[ -n "$output_root" ]] || die "--output is required"
[[ -d "$source_root" ]] || die "source root not found: $source_root"

if command -v dataloader-convert >/dev/null 2>&1; then
  converter_command=(dataloader-convert)
elif command -v rosrun >/dev/null 2>&1; then
  converter_command=(rosrun dataloader dataloader_convert.py)
else
  die "dataloader-convert and rosrun were not found. Install this package or source the catkin workspace."
fi

if [[ ${#sequences[@]} -eq 0 ]]; then
  while IFS= read -r -d '' dir; do
    sequences+=("$(basename "$dir")")
  done < <(find "$source_root" -mindepth 1 -maxdepth 1 -type d -print0 | sort -z)
fi

[[ ${#sequences[@]} -gt 0 ]] || die "no sequence directories found under: $source_root"

echo ""
echo "========================================"
echo "batch dataloader convert"
echo "dataset      : $dataset"
echo "source root  : $source_root"
echo "output root  : $output_root"
echo "sequences    : ${#sequences[@]}"
echo "overwrite    : $overwrite"
echo "continue err : $continue_on_error"
echo "delete raw   : $delete_source_after_success"
echo "========================================"

success=()
failed=()
skipped=()
deleted=()
delete_failed=()
total=${#sequences[@]}

for index in "${!sequences[@]}"; do
  sequence="${sequences[$index]}"
  source_dir="$source_root/$sequence"
  output_dir="$output_root/$dataset/$sequence"
  number=$((index + 1))

  echo ""
  echo "----------------------------------------"
  echo "[$number/$total] $dataset/$sequence"
  echo "source: $source_dir"
  echo "output: $output_dir"
  echo "----------------------------------------"

  if [[ ! -d "$source_dir" ]]; then
    echo "SKIP: source sequence directory not found"
    skipped+=("$sequence")
    continue
  fi

  if [[ -d "$output_dir" && "$overwrite" -eq 0 ]]; then
    echo "SKIP: converted sequence already exists. Use --overwrite to recreate."
    skipped+=("$sequence")
    continue
  fi

  cmd=(
    "${converter_command[@]}"
    --dataset "$dataset"
    --source "$source_dir"
    --output "$output_root"
    --sequence "$sequence"
  )
  [[ "$overwrite" -eq 1 ]] && cmd+=(--overwrite)
  [[ -n "$link_mode" ]] && cmd+=(--link-mode "$link_mode")
  [[ -n "$start_lidar_frame" ]] && cmd+=(--start-lidar-frame "$start_lidar_frame")
  [[ -n "$end_lidar_frame" ]] && cmd+=(--end-lidar-frame "$end_lidar_frame")

  if "${cmd[@]}"; then
    success+=("$sequence")
    if [[ "$delete_source_after_success" -eq 1 ]]; then
      echo ""
      echo "verify converted copy before deleting source"
      if validate_converted_copy "$output_dir" && delete_source_sequence "$source_dir"; then
        deleted+=("$sequence")
      else
        delete_failed+=("$sequence")
        failed+=("$sequence")
        if [[ "$continue_on_error" -eq 0 ]]; then
          echo ""
          echo "STOP: source deletion failed at $dataset/$sequence"
          break
        fi
      fi
    fi
  else
    failed+=("$sequence")
    if [[ "$continue_on_error" -eq 0 ]]; then
      echo ""
      echo "STOP: conversion failed at $dataset/$sequence"
      break
    fi
  fi
done

echo ""
echo "========================================"
echo "batch summary"
echo "dataset : $dataset"
echo "output  : $output_root/$dataset"
echo "success : ${#success[@]}"
printf '  %s\n' "${success[@]:-}" | sed '/^  $/d'
echo "skipped : ${#skipped[@]}"
printf '  %s\n' "${skipped[@]:-}" | sed '/^  $/d'
echo "failed  : ${#failed[@]}"
printf '  %s\n' "${failed[@]:-}" | sed '/^  $/d'
echo "deleted : ${#deleted[@]}"
printf '  %s\n' "${deleted[@]:-}" | sed '/^  $/d'
echo "delete failed : ${#delete_failed[@]}"
printf '  %s\n' "${delete_failed[@]:-}" | sed '/^  $/d'
echo "========================================"

if [[ ${#failed[@]} -gt 0 ]]; then
  exit 1
fi

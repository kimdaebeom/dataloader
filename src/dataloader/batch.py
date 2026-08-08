"""Python batch-conversion API."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path

from .common import validate_path_component
from .converter import LINK_MODES, convert_dataset
from .converters import available_converters, get_converter


@dataclass
class BatchConversionResult:
    dataset: str
    output_root: Path
    successful: list = field(default_factory=list)
    skipped: list = field(default_factory=list)
    failed: dict = field(default_factory=dict)
    results: dict = field(default_factory=dict)

    @property
    def ok(self):
        return not self.failed


def convert_many(
    dataset,
    source_root,
    output_root,
    sequences=None,
    workers=1,
    continue_on_error=True,
    **convert_options
):
    """Convert multiple sequence directories.

    ``convert_options`` are forwarded to :func:`convert_dataset`. Existing
    outputs are skipped unless ``overwrite=True`` is supplied.
    """
    get_converter(dataset)
    source_root = Path(source_root).expanduser().resolve()
    output_root = Path(output_root).expanduser().resolve()
    if not source_root.is_dir():
        raise FileNotFoundError("source root not found: {}".format(source_root))
    workers = int(workers)
    if workers < 1:
        raise ValueError("workers must be at least 1")
    if workers > 1 and not continue_on_error:
        raise ValueError("continue_on_error=False requires workers=1")
    if sequences is None:
        sequences = sorted(path.name for path in source_root.iterdir() if path.is_dir())
    else:
        sequences = [
            validate_path_component(sequence, "sequence")
            for sequence in sequences
        ]

    overwrite = bool(convert_options.get("overwrite", False))
    result = BatchConversionResult(dataset=dataset, output_root=output_root)
    pending = []
    for sequence in sequences:
        source = source_root / sequence
        destination = output_root / dataset / sequence
        if not source.is_dir():
            result.skipped.append(sequence)
        elif destination.exists() and not overwrite:
            result.skipped.append(sequence)
        else:
            pending.append(sequence)

    def run(sequence):
        return convert_dataset(
            dataset=dataset,
            source=source_root / sequence,
            output_root=output_root,
            sequence=sequence,
            **convert_options
        )

    if workers == 1:
        for index, sequence in enumerate(pending):
            try:
                converted = run(sequence)
            except Exception as exc:
                result.failed[sequence] = exc
                if not continue_on_error:
                    result.skipped.extend(pending[index + 1 :])
                    break
            else:
                result.successful.append(sequence)
                result.results[sequence] = converted
        return result

    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(run, sequence): sequence for sequence in pending}
        for future in as_completed(futures):
            sequence = futures[future]
            try:
                converted = future.result()
            except Exception as exc:
                result.failed[sequence] = exc
            else:
                result.successful.append(sequence)
                result.results[sequence] = converted
    result.successful.sort()
    result.failed = dict(sorted(result.failed.items()))
    return result


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Convert multiple dataset sequences.")
    parser.add_argument("--dataset", required=True, choices=available_converters())
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("sequences", nargs="*", help="Defaults to every child directory.")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--link-mode", choices=LINK_MODES, default="copy")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--start-lidar-frame", type=int)
    parser.add_argument("--end-lidar-frame", type=int)
    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help="Stop after the first failure; requires --workers 1.",
    )
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.stop_on_error and args.workers != 1:
        parser.error("--stop-on-error requires --workers 1")
    result = convert_many(
        dataset=args.dataset,
        source_root=args.source_root,
        output_root=args.output,
        sequences=args.sequences or None,
        workers=args.workers,
        continue_on_error=not args.stop_on_error,
        link_mode=args.link_mode,
        overwrite=args.overwrite,
        start_lidar_frame=args.start_lidar_frame,
        end_lidar_frame=args.end_lidar_frame,
        verbose=args.verbose,
    )
    print("batch convert: {}".format("OK" if result.ok else "FAILED"))
    print("success: {}".format(", ".join(result.successful) or "-"))
    print("skipped: {}".format(", ".join(result.skipped) or "-"))
    if result.failed:
        print("failed")
        for sequence, error in result.failed.items():
            print("  - {}: {}".format(sequence, error))
    return 0 if result.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())

//! `play_launch log list|prune` (#0098): what the runs under a log
//! directory hold, and removing old ones, wholly or by kind of file.

use crate::{
    cli::options::{LogListArgs, LogPruneArgs},
    util::play_log::{self, ItemClass, Run, Selection, human},
};

/// The classes shown as columns, in order.
const COLUMNS: [ItemClass; 8] = [
    ItemClass::Diagnostics,
    ItemClass::Metrics,
    ItemClass::Events,
    ItemClass::Interception,
    ItemClass::NodeLogs,
    ItemClass::LauncherLog,
    ItemClass::Params,
    ItemClass::Other,
];

pub fn handle_list(args: &LogListArgs) -> eyre::Result<()> {
    let runs = play_log::list_runs(&args.log_dir, true);
    if runs.is_empty() {
        println!("No runs under {}", args.log_dir.display());
        return Ok(());
    }
    print!("{:<24} {:>9}", "run", "total");
    for c in COLUMNS {
        print!(" {:>12}", c.name());
    }
    println!();
    for r in &runs {
        print!(
            "{:<24} {:>9}",
            format!("{}{}", r.name, if r.in_use { "*" } else { "" }),
            human(r.total())
        );
        for c in COLUMNS {
            print!(" {:>12}", r.sizes.get(&c).map_or("-".into(), |b| human(*b)));
        }
        println!();
    }
    let total: u64 = runs.iter().map(Run::total).sum();
    println!(
        "\n{} run(s), {} in {}",
        runs.len(),
        human(total),
        args.log_dir.display()
    );
    if runs.iter().any(|r| r.in_use) {
        println!("* a play_launch process is still writing it; `log prune` leaves it alone");
    }
    Ok(())
}

pub fn handle_prune(args: &LogPruneArgs) -> eyre::Result<()> {
    let sel = Selection {
        before: args
            .before
            .as_deref()
            .map(|b| play_log::parse_before(b, chrono::Local::now()))
            .transpose()
            .map_err(|e| eyre::eyre!("--before {e}"))?,
        keep: args.keep,
        max_total: args.max_total_mb.map(|mb| mb.saturating_mul(1_000_000)),
    };
    if sel.is_empty() {
        eyre::bail!("say which runs: --before <WHEN>, --keep <N> and/or --max-total-mb <MB>");
    }
    let runs = play_log::list_runs(&args.log_dir, true);
    let chosen = play_log::select(&runs, &sel);
    let what = if args.classes.is_empty() {
        "whole runs".to_string()
    } else {
        args.classes
            .iter()
            .map(|c| c.name())
            .collect::<Vec<_>>()
            .join(", ")
    };
    let freed = |r: &Run| -> u64 {
        if args.classes.is_empty() {
            r.total()
        } else {
            r.sizes
                .iter()
                .filter(|(c, _)| {
                    args.classes.iter().any(|k| {
                        *k == **c || (*k == ItemClass::Interception && **c == ItemClass::Events)
                    })
                })
                .map(|(_, b)| b)
                .sum()
        }
    };
    let chosen: Vec<&Run> = chosen.into_iter().filter(|r| freed(r) > 0).collect();
    if chosen.is_empty() {
        println!(
            "Nothing to remove under {} ({what}).",
            args.log_dir.display()
        );
        return Ok(());
    }
    let planned: u64 = chosen.iter().map(|r| freed(r)).sum();
    for r in &chosen {
        println!("  {}  {}", r.name, human(freed(r)));
    }
    if !args.yes {
        println!(
            "Would remove {} from {} run(s) ({what}). Nothing removed; add --yes to remove it.",
            human(planned),
            chosen.len()
        );
        return Ok(());
    }
    let mut removed = 0u64;
    let mut failed = 0usize;
    for r in &chosen {
        match play_log::remove(r, &args.classes) {
            Ok(b) => removed += b,
            Err(e) => {
                failed += 1;
                eprintln!("could not remove from {}: {e}", r.path.display());
            }
        }
    }
    println!(
        "Removed {} from {} run(s) ({what}).",
        human(removed),
        chosen.len() - failed
    );
    if failed > 0 {
        eyre::bail!("{failed} run(s) could not be pruned");
    }
    Ok(())
}

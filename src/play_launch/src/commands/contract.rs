//! `play_launch contract eject` — copy a package's provider contract (and
//! matching scheduling platform file, if any) into the overlay tree so a
//! user can edit them without ever touching `/opt` (design §3.3).

use std::path::{Path, PathBuf};

use eyre::{Result, eyre};

use crate::{
    cli::options::ContractEjectArgs,
    ros::{launch_dump::LaunchDump, manifest_loader, sched_loader},
};

pub fn handle_contract_eject(args: &ContractEjectArgs) -> Result<()> {
    let launch_path =
        super::launch::resolve_launch_file(&args.package_or_path, args.launch_file.as_deref())?;

    // No launch arguments needed — eject only cares about the launch tree's
    // ROOT scope (pkg/file/origin path), not the fully-resolved node graph.
    let record = play_launch_parser::parse_launch_file(&launch_path, Default::default())
        .map_err(|e| eyre!("Parser error: {e}"))?;
    let json = serde_json::to_string(&record)?;
    let dump: LaunchDump = serde_json::from_str(&json)?;

    let scope = sched_loader::root_scope(&dump)
        .ok_or_else(|| eyre!("launch dump has no root scope (empty or malformed record)"))?;

    let provider_contract = manifest_loader::resolve_provider_path(scope).filter(|p| p.exists());
    let provider_platform =
        sched_loader::resolve_platform_provider_path(scope, &args.target).filter(|p| p.exists());

    if provider_contract.is_none() && provider_platform.is_none() {
        let dir = scope
            .origin
            .as_ref()
            .and_then(|o| o.path.as_deref())
            .and_then(|p| Path::new(p).parent())
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| "<unknown directory>".to_string());
        return Err(eyre!(
            "package ships neither a provider contract nor a provider platform file for \
             target `{}` next to the launch file in {dir}",
            args.target
        ));
    }

    // Resolve the overlay root to eject into: `--into` always wins (created
    // unconditionally — it's user config space, not `/opt`, so no
    // permission prompt is needed); otherwise fall back to the same
    // discovery `check --contracts` uses, erroring if nothing exists yet.
    let overlay_root: PathBuf = match &args.into {
        Some(p) => p.clone(),
        None => manifest_loader::discover_overlay_root(None).ok_or_else(|| {
            eyre!(
                "no overlay root found and no --into given; pass --into <dir> \
                 (e.g. --into ~/.config/play_launch/contracts) or set $PLAY_LAUNCH_CONTRACTS"
            )
        })?,
    };

    let contract_dest = provider_contract
        .as_ref()
        .and_then(|_| manifest_loader::resolve_overlay_path(scope, &overlay_root));
    let platform_dest = provider_platform.as_ref().and_then(|_| {
        sched_loader::resolve_platform_overlay_path(scope, &overlay_root, &args.target)
    });

    // Refuse to clobber existing overlay files unless --force — checked for
    // *every* candidate destination before copying anything, so a conflict
    // on one file doesn't leave the other half-ejected.
    if !args.force {
        for dest in [&contract_dest, &platform_dest].into_iter().flatten() {
            if dest.exists() {
                return Err(eyre!(
                    "refusing to overwrite existing overlay file {} (pass --force to overwrite)",
                    dest.display()
                ));
            }
        }
    }

    let mut ejected_any = false;
    if let (Some(src), Some(dest)) = (&provider_contract, &contract_dest) {
        std::fs::create_dir_all(
            dest.parent().ok_or_else(|| {
                eyre!("contract destination {} has no parent dir", dest.display())
            })?,
        )?;
        std::fs::copy(src, dest)?;
        eprintln!("Ejected contract: {} -> {}", src.display(), dest.display());
        ejected_any = true;
    }
    if let (Some(src), Some(dest)) = (&provider_platform, &platform_dest) {
        std::fs::create_dir_all(
            dest.parent().ok_or_else(|| {
                eyre!("platform destination {} has no parent dir", dest.display())
            })?,
        )?;
        std::fs::copy(src, dest)?;
        eprintln!(
            "Ejected platform file: {} -> {}",
            src.display(),
            dest.display()
        );
        ejected_any = true;
    }

    if !ejected_any {
        // Shouldn't be reachable given the guard above, but keep the exit
        // code honest if the code above is ever refactored.
        return Err(eyre!("nothing was ejected"));
    }

    Ok(())
}

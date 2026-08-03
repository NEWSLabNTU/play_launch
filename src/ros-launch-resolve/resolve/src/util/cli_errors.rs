//! A user-facing error report: the message and its causes, nothing else.
//!
//! # The problem this solves
//!
//! `fn main() -> eyre::Result<()>` reports a failure by `Debug`-formatting
//! the error, and eyre's DEFAULT handler appends the source location it
//! captured:
//!
//! ```text
//! Error: Launch file not found: /nonexistent/foo.launch.xml
//!
//! Location:
//!     src/ros-launch-resolve/resolve/src/verbs/mod.rs:93:24
//! ```
//!
//! That footer is a stack-trace artifact aimed at whoever wrote the code, and
//! it leaks an internal path into the output of a shipped product. Worse for
//! this repo specifically: the verbs live in `src/ros-launch-resolve/`, so
//! **every** error from `resolve`, `dump`, `check`, `plot` and `contract
//! eject` printed the name of the developer-only binary to a user who was
//! only ever given `play_launch` — on the one surface (`stderr` of a failing
//! run) that no `--help` audit can see.
//!
//! # What replaces it
//!
//! [`install`] registers a handler that renders the message and the `Caused
//! by:` chain and stops there — the same information, minus the file path.
//! Both CLIs call it as the first statement of `main`.

use std::fmt;

/// Renders an error as message + `Caused by:` chain, with no location footer.
struct PlainHandler;

impl eyre::EyreHandler for PlainHandler {
    fn debug(
        &self,
        error: &(dyn std::error::Error + 'static),
        f: &mut fmt::Formatter<'_>,
    ) -> fmt::Result {
        // `{:#?}` still means "give me the raw derived Debug" — honour it so
        // a developer inspecting an error keeps the escape hatch.
        if f.alternate() {
            return fmt::Debug::fmt(error, f);
        }

        write!(f, "{error}")?;

        let mut source = error.source();
        if source.is_some() {
            write!(f, "\n\nCaused by:")?;
        }
        let mut i = 0usize;
        while let Some(cause) = source {
            write!(f, "\n{i:>5}: {cause}")?;
            source = cause.source();
            i += 1;
        }
        Ok(())
    }
}

/// Install the plain report as the process-wide eyre hook.
///
/// Idempotent and infallible from the caller's point of view: `eyre::set_hook`
/// errors only if a hook was already installed, which is not a condition a CLI
/// should abort over.
pub fn install() {
    let _ = eyre::set_hook(Box::new(|_| Box::new(PlainHandler)));
}

#[cfg(test)]
mod tests {
    /// The report must carry the message and the cause chain but no
    /// `Location:` footer — and, specifically, no repo path naming the
    /// developer binary.
    ///
    /// The hook is process-global and `set_hook` only succeeds once, so this
    /// formats the handler directly rather than installing it, which would
    /// make the test order-dependent against any other test in this binary.
    #[test]
    fn a_plain_report_has_no_location_and_no_internal_path() {
        struct Wrap<'a>(&'a (dyn std::error::Error + 'static));
        impl std::fmt::Debug for Wrap<'_> {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                eyre::EyreHandler::debug(&super::PlainHandler, self.0, f)
            }
        }

        let err = std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "Launch file not found: /nonexistent/foo.launch.xml",
        );
        let rendered = format!("{:?}", Wrap(&err));

        assert!(rendered.contains("Launch file not found"), "{rendered}");
        assert!(!rendered.contains("Location:"), "{rendered}");
        assert!(!rendered.contains("ros-launch-resolve"), "{rendered}");
    }
}

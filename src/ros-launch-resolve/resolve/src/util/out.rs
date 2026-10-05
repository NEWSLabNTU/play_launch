//! What `check` writes to the terminal, made safe for a log (phase 85 I5).
//!
//! The checker's report used to carry U+2192 in every route, em dashes in
//! its messages and U+2500 rules around its sections, emitted ANSI colour
//! into a pipe, and ran single lines to 463 characters. A CI log, an ASCII
//! document and `grep` all had to undo that (the island's
//! `check-contracts.sh` stripped the colour with `sed`).
//!
//! Every line `check` prints goes through [`emit`], which applies the
//! process-wide settings [`configure`] stored:
//!
//! - **ASCII** ([`to_ascii`]): `->` for an arrow, `--` for an em dash,
//!   `-`/`|`/`+` for box drawing, and `?` for anything else outside ASCII.
//!   `check` turns it on with `--ascii`, and on its own when stderr is not a
//!   terminal.
//! - **width** ([`wrap`]): lines longer than `--width N` are broken at a
//!   space and continued with the line's indentation plus four spaces. Off
//!   unless asked for: wrapping by default would split the substrings a
//!   script greps for.
//!
//! Colour is decided by the caller ([`color_wanted`]): on only when stderr
//! is a terminal and `NO_COLOR` is unset or empty. The defaults are "off",
//! so a verb that never calls [`configure`] prints exactly what it did
//! before.

use std::{
    borrow::Cow,
    io::Write,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

static ASCII: AtomicBool = AtomicBool::new(false);
static WIDTH: AtomicUsize = AtomicUsize::new(0);

/// Set the output policy for this process. `width` 0 = no wrapping.
pub fn configure(ascii: bool, width: usize) {
    ASCII.store(ascii, Ordering::Relaxed);
    WIDTH.store(width, Ordering::Relaxed);
}

/// Is ASCII-only output in force?
pub fn ascii() -> bool {
    ASCII.load(Ordering::Relaxed)
}

/// Is any policy in force (ASCII or a width)? When not, [`render`] is the
/// identity.
pub fn active() -> bool {
    ascii() || WIDTH.load(Ordering::Relaxed) > 0
}

/// Should a stream that `is_terminal` carry ANSI colour? `NO_COLOR`
/// (<https://no-color.org>), when set to anything non-empty, says no.
pub fn color_wanted(is_terminal: bool) -> bool {
    is_terminal && std::env::var_os("NO_COLOR").is_none_or(|v| v.is_empty())
}

/// Transliterate `s` to ASCII. Box drawing becomes `-`, `|` or `+`; the
/// arrows and dashes the checker uses get their ASCII spellings; any other
/// non-ASCII character becomes `?`, so the result never holds a byte >= 0x80.
pub fn to_ascii(s: &str) -> Cow<'_, str> {
    if s.is_ascii() {
        return Cow::Borrowed(s);
    }
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        if c.is_ascii() {
            out.push(c);
            continue;
        }
        let rep: &str = match c {
            '\u{2192}' | '\u{27F6}' => "->",
            '\u{2190}' | '\u{27F5}' => "<-",
            '\u{2194}' => "<->",
            '\u{21D2}' => "=>",
            '\u{2014}' | '\u{2015}' => "--",
            '\u{2013}' | '\u{2010}' | '\u{2011}' | '\u{2212}' => "-",
            '\u{2264}' => "<=",
            '\u{2265}' => ">=",
            '\u{2260}' => "!=",
            '\u{2248}' => "~=",
            '\u{00B1}' => "+/-",
            '\u{00D7}' => "x",
            '\u{2026}' => "...",
            '\u{00B7}' | '\u{2022}' | '\u{2219}' => "*",
            '\u{2018}' | '\u{2019}' | '\u{2032}' => "'",
            '\u{201C}' | '\u{201D}' | '\u{2033}' => "\"",
            '\u{00B5}' | '\u{03BC}' => "u",
            '\u{221E}' => "inf",
            '\u{00A0}' => " ",
            // Box drawing (U+2500..U+257F): horizontals, verticals, and every
            // corner or junction.
            '\u{2500}' | '\u{2501}' | '\u{2504}' | '\u{2505}' | '\u{2508}' | '\u{2509}'
            | '\u{254C}' | '\u{254D}' | '\u{2550}' | '\u{2574}' | '\u{2576}' | '\u{2578}'
            | '\u{257A}' | '\u{257C}' | '\u{257E}' => "-",
            '\u{2502}' | '\u{2503}' | '\u{2506}' | '\u{2507}' | '\u{250A}' | '\u{250B}'
            | '\u{254E}' | '\u{254F}' | '\u{2551}' | '\u{2575}' | '\u{2577}' | '\u{2579}'
            | '\u{257B}' | '\u{257D}' | '\u{257F}' => "|",
            '\u{2500}'..='\u{257F}' => "+",
            _ => "?",
        };
        out.push_str(rep);
    }
    Cow::Owned(out)
}

/// Visible width of `s`: characters, not counting ANSI escape sequences.
fn visible_len(s: &str) -> usize {
    let mut n = 0;
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' {
            if chars.peek() == Some(&'[') {
                chars.next();
                for d in chars.by_ref() {
                    if d.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
            continue;
        }
        n += 1;
    }
    n
}

/// Break every line of `s` longer than `width` visible characters at a
/// space; a continuation takes the line's indentation plus four spaces. A
/// word longer than the room left is cut. `width` 0 returns `s` unchanged.
pub fn wrap(s: &str, width: usize) -> Cow<'_, str> {
    if width == 0 || s.lines().all(|l| visible_len(l) <= width) {
        return Cow::Borrowed(s);
    }
    let mut out = String::with_capacity(s.len() + s.len() / width.max(1) * 8);
    let mut first = true;
    for line in s.split('\n') {
        if !first {
            out.push('\n');
        }
        first = false;
        if visible_len(line) <= width {
            out.push_str(line);
            continue;
        }
        let indent: String = line.chars().take_while(|c| *c == ' ').collect();
        // A continuation must leave room for at least some text.
        let cont = if indent.len() + 4 < width / 2 {
            format!("{indent}    ")
        } else {
            String::new()
        };
        let mut cur = String::new();
        let mut cur_len = 0usize;
        let mut has_word = false;
        let flush = |out: &mut String, cur: &mut String, cur_len: &mut usize| {
            out.push_str(cur.trim_end());
            out.push('\n');
            cur.clear();
            cur.push_str(&cont);
            *cur_len = cont.len();
        };
        for word in line.split(' ') {
            let w = visible_len(word);
            let sep = usize::from(has_word);
            if cur_len + sep + w <= width {
                if has_word {
                    cur.push(' ');
                }
                cur.push_str(word);
                cur_len += sep + w;
                has_word = true;
                continue;
            }
            if has_word {
                flush(&mut out, &mut cur, &mut cur_len);
            }
            // The word alone may still not fit: cut it.
            let mut rest: Vec<char> = word.chars().collect();
            while cur_len + rest.len() > width {
                let room = (width - cur_len).max(1);
                let head: String = rest.drain(..room.min(rest.len())).collect();
                cur.push_str(&head);
                flush(&mut out, &mut cur, &mut cur_len);
            }
            cur_len += rest.len();
            cur.extend(rest);
            has_word = true;
        }
        out.push_str(cur.trim_end());
    }
    Cow::Owned(out)
}

/// Apply the configured policy to `s`.
pub fn render(s: &str) -> String {
    let s = if ascii() {
        to_ascii(s)
    } else {
        Cow::Borrowed(s)
    };
    wrap(&s, WIDTH.load(Ordering::Relaxed)).into_owned()
}

/// Write `s` to stderr under the configured policy, without a newline.
pub fn emit(s: &str) {
    let _ = std::io::stderr().lock().write_all(render(s).as_bytes());
}

/// [`emit`] plus a newline.
pub fn emit_line(s: &str) {
    let mut r = render(s);
    r.push('\n');
    let _ = std::io::stderr().lock().write_all(r.as_bytes());
}

/// `eprintln!` through [`emit_line`].
#[macro_export]
macro_rules! say {
    () => { $crate::util::out::emit_line("") };
    ($($t:tt)*) => { $crate::util::out::emit_line(&format!($($t)*)) };
}

/// `eprint!` through [`emit`].
#[macro_export]
macro_rules! say_raw {
    ($($t:tt)*) => { $crate::util::out::emit(&format!($($t)*)) };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_checkers_unicode_becomes_ascii() {
        let s = "\u{2500}\u{2500} Cross-scope \u{2500}\u{2500} a \u{2192} b \u{2014} c \u{2265} 3 \u{250C}\u{2502}\u{2514}";
        let a = to_ascii(s);
        assert_eq!(a, "-- Cross-scope -- a -> b -- c >= 3 +|+");
        assert!(a.is_ascii());
        assert!(to_ascii("\u{4E2D}").is_ascii());
    }

    #[test]
    fn wrap_breaks_at_spaces_and_indents_the_continuation() {
        let line = format!("  {}", ["word"; 40].join(" "));
        let w = wrap(&line, 40);
        for l in w.lines() {
            assert!(l.len() <= 40, "{l:?} in\n{w}");
        }
        assert!(w.lines().nth(1).unwrap().starts_with("      word"), "{w}");
        assert_eq!(w.split_whitespace().count(), 40, "no word lost:\n{w}");
    }

    #[test]
    fn wrap_cuts_a_word_longer_than_the_width() {
        let long = "x".repeat(100);
        let w = wrap(&long, 30);
        assert!(w.lines().all(|l| l.len() <= 30), "{w}");
        assert_eq!(w.chars().filter(|c| *c == 'x').count(), 100);
    }

    #[test]
    fn wrap_ignores_colour_codes_when_measuring() {
        let s = "\u{1b}[1;31merror\u{1b}[0m: short";
        assert_eq!(wrap(s, 20), s);
    }

    #[test]
    fn zero_width_and_short_lines_are_untouched() {
        assert_eq!(wrap("a b c", 0), "a b c");
        assert_eq!(wrap("a b c\nd", 10), "a b c\nd");
    }
}

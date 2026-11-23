//! Only our own page may change anything (phase 105 W1, issue #0111).
//!
//! CORS does not protect the action endpoints. A body-less `POST` is a
//! "simple request": any web page the user has open can send one with
//! `fetch(url, {method: 'POST', mode: 'no-cors'})`, and CORS only stops that
//! page from reading the answer, after the node has already been stopped.
//! And CORS does nothing against DNS rebinding, where a hostile name resolves
//! to 127.0.0.1 and the browser treats our server as that page's own origin.
//!
//! Two checks, on every request:
//!
//! - **`Host`** must name this server when it is bound to a specific
//!   address: the bound address, `localhost`, or a loopback address, with
//!   the bound port or none. A rebinding page's requests carry ITS name, so they are
//!   refused. Bound to an unspecified address (`0.0.0.0`, `::`) the server
//!   cannot know every name it is reached by, and the check is skipped.
//! - **`Origin`**, on anything but `GET`/`HEAD`/`OPTIONS`, must be this
//!   server's own origin: the scheme plus the request's own `Host`. A
//!   cross-site page's request carries the page's origin. Browsers send
//!   `Origin` on every `POST`; a request with none (curl, scripts) did not
//!   come from a page and passes. `Sec-Fetch-Site: cross-site` is refused
//!   too, for a browser that sends that and not `Origin`.
//!
//! And, when a token is configured (`--web-token`), every request must carry
//! it: in the `X-Play-Launch-Token` header, in the `play_launch_token`
//! cookie, or once as `?token=` in the URL, which answers with that cookie
//! set (`HttpOnly; SameSite=Strict`), so the page and its `EventSource`
//! streams carry it from then on and another site's requests never do. The
//! checks above cannot stop a page on another host from READING an exposed
//! server; the token does.

use axum::{
    extract::Request,
    http::{Method, StatusCode, header},
    middleware::Next,
    response::{IntoResponse, Response},
};
use std::net::IpAddr;

/// What the server was bound to, for judging `Host`.
#[derive(Debug, Clone)]
pub struct OriginPolicy {
    /// `None` when bound to an unspecified address: any `Host` is accepted.
    allowed_hosts: Option<Vec<String>>,
    /// Required on every request when set.
    token: Option<String>,
}

/// The cookie the token is kept in once the page has presented it.
pub const TOKEN_COOKIE: &str = "play_launch_token";
/// The header a script may send the token in.
pub const TOKEN_HEADER: &str = "x-play-launch-token";

/// How a request presented the token.
#[derive(Debug, PartialEq, Eq)]
pub enum TokenVerdict {
    /// No token is configured, or the request carried it in a header or
    /// the cookie.
    Pass,
    /// It came in the URL: answer with the cookie set.
    PassSetCookie,
    /// Missing or wrong.
    Refuse,
}

/// Equal without returning early at the first differing byte.
fn same_secret(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

impl OriginPolicy {
    pub fn new(bind_ip: IpAddr, port: u16) -> Self {
        if bind_ip.is_unspecified() {
            return Self {
                allowed_hosts: None,
                token: None,
            };
        }
        let mut names = vec![
            "localhost".to_string(),
            "127.0.0.1".to_string(),
            "[::1]".to_string(),
        ];
        let bound = match bind_ip {
            IpAddr::V4(v4) => v4.to_string(),
            IpAddr::V6(v6) => format!("[{v6}]"),
        };
        if !names.contains(&bound) {
            names.push(bound);
        }
        let mut allowed = Vec::with_capacity(names.len() * 2);
        for name in names {
            // A bare name too: a client may leave the port out, and what a
            // rebinding page cannot change is the NAME it sends.
            allowed.push(format!("{name}:{port}"));
            allowed.push(name);
        }
        Self {
            allowed_hosts: Some(allowed),
            token: None,
        }
    }

    /// Require `token` on every request (`None` or empty: no token).
    pub fn with_token(mut self, token: Option<String>) -> Self {
        self.token = token.filter(|t| !t.is_empty());
        self
    }

    /// Judge the token a request presented in a header, in its `Cookie`
    /// header, or in its query string.
    pub fn token_verdict(
        &self,
        header: Option<&str>,
        cookies: Option<&str>,
        query: Option<&str>,
    ) -> TokenVerdict {
        let Some(token) = &self.token else {
            return TokenVerdict::Pass;
        };
        if header.is_some_and(|h| same_secret(h, token)) {
            return TokenVerdict::Pass;
        }
        let from_cookie = cookies.into_iter().flat_map(|c| c.split(';')).any(|kv| {
            kv.trim()
                .split_once('=')
                .is_some_and(|(k, v)| k == TOKEN_COOKIE && same_secret(v, token))
        });
        if from_cookie {
            return TokenVerdict::Pass;
        }
        let from_query = query.into_iter().flat_map(|q| q.split('&')).any(|kv| {
            kv.split_once('=')
                .is_some_and(|(k, v)| k == "token" && same_secret(v, token))
        });
        if from_query {
            return TokenVerdict::PassSetCookie;
        }
        TokenVerdict::Refuse
    }

    /// Whether the server is reachable from other hosts (an unspecified
    /// bind address).
    pub fn exposed(&self) -> bool {
        self.allowed_hosts.is_none()
    }

    /// `Err` names the rule a request broke.
    pub fn check(
        &self,
        method: &Method,
        host: Option<&str>,
        origin: Option<&str>,
        fetch_site: Option<&str>,
    ) -> Result<(), &'static str> {
        if let Some(allowed) = &self.allowed_hosts {
            let Some(host) = host else {
                return Err("request has no Host header");
            };
            let host = host.to_ascii_lowercase();
            if !allowed.contains(&host) {
                return Err("Host does not name this server (DNS rebinding?)");
            }
        }
        if matches!(*method, Method::GET | Method::HEAD | Method::OPTIONS) {
            return Ok(());
        }
        if fetch_site == Some("cross-site") {
            return Err("cross-site request refused");
        }
        match origin {
            None => Ok(()),
            Some(origin) => {
                let Some(host) = host else {
                    return Err("request has no Host header");
                };
                let host = host.to_ascii_lowercase();
                let origin = origin.to_ascii_lowercase();
                let own = origin
                    .strip_prefix("http://")
                    .or_else(|| origin.strip_prefix("https://"))
                    .is_some_and(|rest| rest == host);
                if own {
                    Ok(())
                } else {
                    Err("Origin is not this server's page")
                }
            }
        }
    }
}

/// Axum middleware applying [`OriginPolicy::check`].
pub async fn guard(policy: std::sync::Arc<OriginPolicy>, req: Request, next: Next) -> Response {
    let headers = req.headers();
    let text = |name: header::HeaderName| headers.get(name).and_then(|v| v.to_str().ok());
    let verdict = policy.check(
        req.method(),
        text(header::HOST),
        text(header::ORIGIN),
        headers.get("sec-fetch-site").and_then(|v| v.to_str().ok()),
    );
    if let Err(rule) = verdict {
        tracing::warn!(
            "web UI refused {} {}: {}",
            req.method(),
            req.uri().path(),
            rule
        );
        return (StatusCode::FORBIDDEN, format!("refused: {rule}\n")).into_response();
    }
    let token = policy.token_verdict(
        headers.get(TOKEN_HEADER).and_then(|v| v.to_str().ok()),
        text(header::COOKIE),
        req.uri().query(),
    );
    match token {
        TokenVerdict::Pass => next.run(req).await,
        TokenVerdict::PassSetCookie => {
            let token = policy.token.clone().unwrap_or_default();
            let mut response = next.run(req).await;
            if let Ok(value) = header::HeaderValue::from_str(&format!(
                "{TOKEN_COOKIE}={token}; Path=/; HttpOnly; SameSite=Strict"
            )) {
                response.headers_mut().append(header::SET_COOKIE, value);
            }
            response
        }
        TokenVerdict::Refuse => {
            let rule = "this web UI needs its token: open it as /?token=<TOKEN>";
            tracing::warn!(
                "web UI refused {} {}: {}",
                req.method(),
                req.uri().path(),
                rule
            );
            (StatusCode::FORBIDDEN, format!("refused: {rule}\n")).into_response()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn local() -> OriginPolicy {
        OriginPolicy::new("127.0.0.1".parse().unwrap(), 8080)
    }

    #[test]
    fn the_page_itself_may_post() {
        let p = local();
        for host in ["127.0.0.1:8080", "localhost:8080", "[::1]:8080"] {
            let origin = format!("http://{host}");
            assert_eq!(
                p.check(
                    &Method::POST,
                    Some(host),
                    Some(&origin),
                    Some("same-origin")
                ),
                Ok(())
            );
        }
    }

    #[test]
    fn a_foreign_page_may_not_post() {
        let p = local();
        assert!(
            p.check(
                &Method::POST,
                Some("127.0.0.1:8080"),
                Some("https://evil.example"),
                None
            )
            .is_err()
        );
        assert!(
            p.check(
                &Method::POST,
                Some("127.0.0.1:8080"),
                None,
                Some("cross-site")
            )
            .is_err()
        );
        // Another local port is another origin.
        assert!(
            p.check(
                &Method::POST,
                Some("127.0.0.1:8080"),
                Some("http://127.0.0.1:3000"),
                None
            )
            .is_err()
        );
    }

    #[test]
    fn a_rebound_name_may_not_even_read() {
        let p = local();
        assert!(
            p.check(&Method::GET, Some("evil.example:8080"), None, None)
                .is_err()
        );
        assert!(p.check(&Method::GET, None, None, None).is_err());
        assert_eq!(
            p.check(&Method::GET, Some("LocalHost:8080"), None, None),
            Ok(())
        );
    }

    #[test]
    fn a_script_without_origin_may_post() {
        assert_eq!(
            local().check(&Method::POST, Some("localhost:8080"), None, None),
            Ok(())
        );
    }

    #[test]
    fn an_exposed_server_accepts_any_host_but_still_only_its_own_page() {
        for ip in ["0.0.0.0", "::"] {
            let p = OriginPolicy::new(ip.parse().unwrap(), 8080);
            assert!(p.exposed());
            assert_eq!(
                p.check(
                    &Method::POST,
                    Some("robot1:8080"),
                    Some("http://robot1:8080"),
                    None
                ),
                Ok(())
            );
            assert!(
                p.check(
                    &Method::POST,
                    Some("robot1:8080"),
                    Some("http://evil.example"),
                    None
                )
                .is_err()
            );
        }
    }

    #[test]
    fn no_token_configured_passes_everything() {
        assert_eq!(local().token_verdict(None, None, None), TokenVerdict::Pass);
        let p = local().with_token(Some(String::new()));
        assert_eq!(p.token_verdict(None, None, None), TokenVerdict::Pass);
    }

    #[test]
    fn a_token_is_taken_from_header_cookie_or_url() {
        let p = local().with_token(Some("s3cret".into()));
        assert_eq!(p.token_verdict(None, None, None), TokenVerdict::Refuse);
        assert_eq!(
            p.token_verdict(Some("s3cret"), None, None),
            TokenVerdict::Pass
        );
        assert_eq!(
            p.token_verdict(None, Some("theme=dark; play_launch_token=s3cret"), None),
            TokenVerdict::Pass
        );
        assert_eq!(
            p.token_verdict(None, None, Some("view=health&token=s3cret")),
            TokenVerdict::PassSetCookie
        );
    }

    #[test]
    fn a_wrong_or_partial_token_is_refused() {
        let p = local().with_token(Some("s3cret".into()));
        for wrong in ["s3cre", "s3cret!", "S3CRET", ""] {
            assert_eq!(
                p.token_verdict(Some(wrong), None, None),
                TokenVerdict::Refuse
            );
            assert_eq!(
                p.token_verdict(None, Some(&format!("play_launch_token={wrong}")), None),
                TokenVerdict::Refuse
            );
            assert_eq!(
                p.token_verdict(None, None, Some(&format!("token={wrong}"))),
                TokenVerdict::Refuse
            );
        }
        // Another cookie holding the value is not the token cookie.
        assert_eq!(
            p.token_verdict(None, Some("other=s3cret"), None),
            TokenVerdict::Refuse
        );
    }

    #[test]
    fn a_specific_bind_address_is_its_own_host() {
        let p = OriginPolicy::new("192.168.1.5".parse().unwrap(), 9000);
        assert!(!p.exposed());
        assert_eq!(
            p.check(&Method::GET, Some("192.168.1.5:9000"), None, None),
            Ok(())
        );
        assert!(
            p.check(&Method::GET, Some("192.168.1.6:9000"), None, None)
                .is_err()
        );
    }
}

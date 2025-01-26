mod key_owned;
mod key_type;
mod non_private_key;
mod relative_key;

pub use key_owned::KeyOwned;
pub use key_type::{Key, KeyKind, StripKeyPrefix};
pub use non_private_key::NonPrivateKeyOwned;
pub use relative_key::RelativeKeyOwned;

use pomsky_macro::pomsky;
use regex::Regex;
use std::sync::LazyLock;

pub static RE_KEY: LazyLock<Regex> = LazyLock::new(|| {
    let rule = pomsky! {
        let token = [ascii_alpha '_'] [ascii_word]*;
        let token_seq = token ('/' token)*;
        ^ (('~'? '/' token_seq?) | token_seq) $
    };
    Regex::new(rule).expect("invalid regex for key")
});

#[cfg(test)]
mod tests {
    use super::KeyOwned;
    use std::str::FromStr;

    #[test]
    fn suceed_on_valid_keys() {
        let valid_keys = [
            "/",
            "/x",
            "/xy",
            "/x/y",
            "/xx/y/zzz",
            "xyz",
            "xy",
            "x/y/z",
            "~/",
            "~/xx",
            "~/xx/yy",
        ];

        for path in valid_keys {
            let _: KeyOwned = path.parse().unwrap();
        }
    }

    #[test]
    fn fail_on_invalid_keys() {
        let invalid_keys = [
            "", "/x/", "//", "/x//y", "xx//y", "xx//", "yy/", "~", "a/~/b", "~//", "~/xx/yy/",
        ];

        for path in invalid_keys {
            let Err(_) = KeyOwned::from_str(path) else {
                panic!("the key '{path}' is invalid but the Key is successfully constructed")
            };
        }
    }
}

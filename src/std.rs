use crate::{acc_config, Error};
use std::{error, fmt};

impl<E: core::fmt::Debug> error::Error for Error<E> {}

impl<E: core::fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl error::Error for acc_config::Error {}

impl fmt::Display for acc_config::Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

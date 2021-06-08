use crate::Error;
use std::{error, fmt};

impl<E: core::fmt::Debug> error::Error for Error<E> {}

impl<E: core::fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

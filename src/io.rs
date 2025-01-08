//Helpers to do string interpolation since it's no_std
pub struct Cursor<'a> {
    pub buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    pub fn new(buf: &'a mut [u8]) -> Cursor<'a> {
        Cursor { buf, pos: 0 }
    }

    pub fn just_the_bytes(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.pos]).unwrap()
    }

    pub fn clear(&mut self) {
        for i in 0..self.buf.len() {
            self.buf[i] = 0;
        }
        self.pos = 0;
    }
}

impl<'a> core::fmt::Write for Cursor<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let len = s.as_bytes().len();
        if len < self.buf.len() - self.pos {
            self.buf[self.pos..self.pos + len].clone_from_slice(s.as_bytes());
            self.pos += len;
            Ok(())
        } else {
            Err(core::fmt::Error)
        }
    }

    fn write_fmt(&mut self, args: core::fmt::Arguments<'_>) -> core::fmt::Result {
        let writer = core::fmt::write(self, args);
        if writer.is_ok() {
            Ok(())
        } else {
            Err(core::fmt::Error)
        }
    }
}

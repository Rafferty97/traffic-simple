pub fn window<T, const N: usize>(elems: &[T], index: usize) -> [&T; N] {
    let elems = &elems[index..index + N];
    std::array::from_fn(|i| &elems[i])
}

def atom_to_str(value):
    """
    Converts Erlang atoms decoded by erlang-py into Python strings.
    Example: OtpErlangAtom(b'nav2') -> 'nav2'
    """
    if hasattr(value, "value"):
        raw = value.value
        if isinstance(raw, bytes):
            return raw.decode()
        return str(raw)

    text = str(value)

    # fallback if the object is displayed as OtpErlangAtom(b'nav2')
    if "OtpErlangAtom" in text and "b'" in text:
        try:
            return text.split("b'")[1].split("'")[0]
        except Exception:
            return text

    return text


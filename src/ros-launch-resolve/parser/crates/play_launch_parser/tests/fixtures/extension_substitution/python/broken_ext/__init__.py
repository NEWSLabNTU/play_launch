"""A launch extension that cannot be imported, for play_launch's tests (#0099):
`launch` warns and goes on, and a substitution nothing else exposes is then
unknown, with this failure named."""
raise ImportError('broken_ext fails to import on purpose')

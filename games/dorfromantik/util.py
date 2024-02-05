import sys

# 🟩 [g]reen, grass
# 🟨 [f]ield
# 🟦 [w]ater
# ⬛ [s]tation
# 🏠 [h]ouse
# 🌲 [t]ree
# 🌊 ri[v]er
# 🚂 [r]ail
# ❓ [q]uestion

symbol_by_letter = dict(g='🟩', f='🟨', w='🟦', s='⬛', h='🏠', t='🌲', v='🌊', r='🚂', q='❓')

class ExprError(Exception):
    pass

class Expr:
    def __init__(self, pattern, extra):
        if len(pattern) != 6 or not isallcomplexunicode(pattern):
            raise ExprError()
        self.pattern = pattern
        self.extra = extra

    def match(self, pattern):
        for e, p in zip(self.pattern, pattern):
            if e == '❓':
                continue
            if p == '❓':
                continue
            if e == '🟦' and p in '🟩🌊':
                continue
            if p == '🟦' and e in '🟩🌊':
                continue
            if e == '⬛' and p in '🟩🟦🌊🚂':
                continue
            if p == '⬛' and e in '🟩🟦🌊🚂':
                continue
            if e != p:
                return False
        return True

    @property
    def globs(self):
        return self.pattern.count('❓')

    def __repr__(self):
        return f'Expr("{self.pattern}", "{self.extra}")'

def isallcomplexunicode(string):
    return all(len(c.encode()) > 2 for c in string)

def glob_index(string, idx):
    return string[:idx] + '❓' + string[idx+1:]

def joinlines(lines):
    return ''.join(l + '\n' for l in lines)

def load(fn):
    with open(fn) as f:
        lines = f.read().splitlines()
    exprs = []
    for line in lines:
        try:
            exprs.append(Expr(line[:6], line[6:].strip()))
        except ExprError:
            pass
    return exprs

def save_append(fn, exprs):
    lines = []
    for e in exprs:
        lines.append(f'{e.pattern} {e.extra}')
    lines.append('')
    with open(fn, 'a') as f:
        f.write(joinlines(lines))

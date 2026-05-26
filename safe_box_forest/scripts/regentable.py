from pathlib import Path
import sys
sys.path.insert(0, str(Path.cwd()))
import experiments.tro2026_generate_tables as gen

if __name__ == '__main__':
    out = gen.rebuild_table(Path('outputs/paper'))
    path = Path('doc/paper/tro_rewrite_2026/generated/tab_tro_dynamic_rebuild.tex')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(out, encoding='utf-8')
    print('WROTE', path)

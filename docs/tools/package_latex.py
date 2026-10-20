"""Create a self-contained LaTeX ZIP with main.tex at its root."""
import hashlib
import json
import re
import subprocess
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

root = Path(__file__).resolve().parents[1]
report = root / 'report'
results = root / 'results'
tex = (report / 'Bao_cao_23020719_Bai_thuc_hanh_05.tex').read_text()
assets = sorted(set(re.findall(r'\.\./results/([^}]+)', tex)))
main = tex.replace('../results/', 'figures/')
assert not re.findall(r'@[A-Z_]+@', main)
output = results / 'Bao_cao_23020719_Bai05_LaTeX_Prism.zip'
readme = '''# LaTeX báo cáo Bài thực hành 05 — Đỗ Việt Anh, 23020719

File chính: main.tex. Sửa nội dung trực tiếp trong file này.
Gói có đủ hình trong figures/, logo và hỗ trợ tiếng Việt; mọi đường dẫn đều
nằm trong ZIP. Không cần ROS hoặc thư mục results bên ngoài để biên dịch.

Bản cập nhật dùng số liệu run10, đã có link Video demo và GitHub.
Mục 7.2 có ảnh camera OAK-D RGB hiển thị ngay trong RViz.
'''
with ZipFile(output, 'w', ZIP_DEFLATED, compresslevel=6) as archive:
    archive.writestr('main.tex', main)
    archive.writestr('README.md', readme)
    for name in ['vietnamese-unicode.tex', 'uet_logo.png']:
        archive.write(report / name, name)
    for name in assets:
        archive.write(results / name, 'figures/' + name)
with ZipFile(output) as archive:
    assert archive.testzip() is None
    assert 'main.tex' in archive.namelist()
info = subprocess.check_output(['pdfinfo', str(results / 'Bao_cao_23020719_Bai_thuc_hanh_05.pdf')], text=True)
metadata = {'zip': output.name, 'main_file': 'main.tex', 'assets': assets,
            'files': 4 + len(assets),
            'report_pages': int(re.search(r'Pages:\s+(\d+)', info).group(1)),
            'sha256': hashlib.sha256(output.read_bytes()).hexdigest()}
(results / 'latex_prism_package.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
print('LaTeX ZIP:', output)

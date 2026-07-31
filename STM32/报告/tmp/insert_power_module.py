from copy import deepcopy
from pathlib import Path
import shutil
import zipfile

from lxml import etree


ROOT = Path(r"C:\Users\KE\Desktop\2026G\2026G\STM32")
UNPACKED = ROOT / "报告" / "tmp" / "power_module_insert_fresh" / "unpacked"
IMAGE_SOURCE = ROOT / "报告" / "photo" / "344f01b18dc501235a635778d8b3516d_720.png"
IMAGE_TARGET = UNPACKED / "word" / "media" / "image7.png"
OUTPUT = ROOT / "报告" / "tmp" / "周期信号测量分析装置设计报告_电源图更新.docx"

W = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
A = "http://schemas.openxmlformats.org/drawingml/2006/main"
WP = "http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"
PKG_REL = "http://schemas.openxmlformats.org/package/2006/relationships"
NS = {"w": W, "r": R, "a": A, "wp": WP}


def paragraph_text(paragraph):
    return "".join(paragraph.xpath(".//w:t/text()", namespaces=NS))


def replace_runs_with_text(paragraph, text, run_properties=None):
    for child in list(paragraph):
        if child.tag == f"{{{W}}}r":
            paragraph.remove(child)
    run = etree.SubElement(paragraph, f"{{{W}}}r")
    if run_properties is not None:
        run.append(deepcopy(run_properties))
    text_node = etree.SubElement(run, f"{{{W}}}t")
    text_node.text = text


document_path = UNPACKED / "word" / "document.xml"
document_tree = etree.parse(str(document_path))
body = document_tree.find(f".//{{{W}}}body")
paragraphs = body.findall(f"{{{W}}}p")

caption = next(p for p in paragraphs if paragraph_text(p) == "图3-6 系统电源转换与滤波电路")
caption_index = list(body).index(caption)
body_template = paragraphs[71]
image_template = paragraphs[72]
caption_template = paragraphs[73]

description = deepcopy(body_template)
replace_runs_with_text(
    description,
    "模拟调理电路所需负电源由MAX660电荷泵反相产生。器件以+5 V供电，C1构成飞跨电容，C2用于输出滤波，转换后的负电源经接口引出，为运算放大器等双电源器件提供负电压轨。",
)

image_paragraph = deepcopy(image_template)
blip = image_paragraph.find(".//a:blip", namespaces=NS)
blip.set(f"{{{R}}}embed", "rId17")
extent = image_paragraph.find(".//wp:extent", namespaces=NS)
extent.set("cx", "5905500")
extent.set("cy", "2969012")
shape_extent = image_paragraph.find(".//a:xfrm/a:ext", namespaces=NS)
shape_extent.set("cx", "5905500")
shape_extent.set("cy", "2969012")
doc_pr = image_paragraph.find(".//wp:docPr", namespaces=NS)
doc_pr.set("id", "429681059")
doc_pr.set("name", "电源模块原理图")
pic_pr = image_paragraph.find(".//a:graphic/a:graphicData/*/*[1]/*[1]", namespaces=NS)
if pic_pr is not None:
    pic_pr.set("id", "7")
    pic_pr.set("name", "电源模块原理图")

new_caption = deepcopy(caption_template)
first_run_properties = new_caption.find(".//w:r/w:rPr", namespaces=NS)
replace_runs_with_text(
    new_caption,
    "图3-7 MAX660负电源转换电路",
    first_run_properties,
)

body.insert(caption_index + 1, description)
body.insert(caption_index + 2, image_paragraph)
body.insert(caption_index + 3, new_caption)

document_tree.write(
    str(document_path),
    encoding="UTF-8",
    xml_declaration=True,
    standalone=True,
    pretty_print=False,
)
document_bytes = document_path.read_bytes()
document_path.write_bytes(
    document_bytes.replace(
        b"<?xml version='1.0' encoding='UTF-8' standalone='yes'?>",
        b'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
        1,
    )
)

rels_path = UNPACKED / "word" / "_rels" / "document.xml.rels"
rels_tree = etree.parse(str(rels_path))
rels_root = rels_tree.getroot()
if not any(rel.get("Id") == "rId17" for rel in rels_root):
    etree.SubElement(
        rels_root,
        f"{{{PKG_REL}}}Relationship",
        Id="rId17",
        Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image",
        Target="media/image7.png",
    )
rels_tree.write(
    str(rels_path),
    encoding="UTF-8",
    xml_declaration=True,
    standalone=True,
    pretty_print=False,
)
rels_bytes = rels_path.read_bytes()
rels_path.write_bytes(
    rels_bytes.replace(
        b"<?xml version='1.0' encoding='UTF-8' standalone='yes'?>",
        b'<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
        1,
    )
)

shutil.copyfile(IMAGE_SOURCE, IMAGE_TARGET)

if OUTPUT.exists():
    OUTPUT.unlink()
with zipfile.ZipFile(OUTPUT, "w", compression=zipfile.ZIP_DEFLATED) as archive:
    for item in sorted(UNPACKED.rglob("*")):
        if item.is_file():
            archive.write(item, item.relative_to(UNPACKED).as_posix())

print(OUTPUT)

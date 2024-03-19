
from dataclasses import dataclass, field
import shutil
import os
from pathlib import Path
from typing import Any, Mapping, NewType, Sequence
from jinja2 import Environment, FileSystemLoader, select_autoescape, Template
import yaml
import frontmatter
import markdown

"""
Reserved directory names:
- layouts
- partials
- data
- css
- build
- src
"""
SRC_DIR = Path("src")
LAYOUTS_DIR = Path(SRC_DIR, "layouts")
PARTIALS_DIR = Path(SRC_DIR, "partials")
DATA_DIR = Path(SRC_DIR, "data")
CSS_DIR = Path(SRC_DIR, "css")
STATIC_DIR = Path(SRC_DIR, "static")
BUILD_DIR = Path("build")

#TODO: Disable cache as users might not be able to see new posts

@dataclass
class Page:
    path: Path
    """ path relative to the source directory """
    metadata: Mapping[str, Any]
    layout: str
    tags: Sequence[str]
    content: str 

    def __post_init__(self):
        self.metadata["url"] = self.get_url()

    def get_url(self) -> str:
        if self.path.stem == "index":
            return "/"
        return f"/{Path(self.path.parent, self.path.stem).as_posix()}"
        
    def get_path_of_index_html(self) -> Path:
        if self.path.stem == "index":
            return Path(self.path.parent, "index.html")
        return Path(self.path.parent, self.path.stem, "index.html")


"""
Step 1. Gather all Markdown files in the 'src' directory and nested directories
"""
def gather_pages() -> Sequence[Page]:
    pages = []
    for dirpath, _, filenames in os.walk(SRC_DIR):
        if (
            Path(dirpath) in (LAYOUTS_DIR, DATA_DIR, CSS_DIR, STATIC_DIR)
        ):
            continue

        for filename in filenames:
            src_file_path = Path(dirpath, filename)
            with open(src_file_path) as fd:
                assert src_file_path.suffix == ".md"
                page = frontmatter.load(fd)
                tags = page.metadata.get("tags")
                if tags is None:
                    tags = []
                elif isinstance(tags, str):
                    tags = [tags]

                pages.append(
                    Page(
                        path=src_file_path.relative_to(SRC_DIR),
                        content=markdown.markdown(
                            page.content,
                            extensions=[
                                "fenced_code",
                                "codehilite"
                            ],
                        ),
                        metadata=page.metadata,
                        layout=page.metadata.get("layout"),
                        tags=tags,
                    )
                )

    return pages

"""
Step 2. Gather all template files. The script expects them to live inside the 
        'layouts' directory. Returns a mapping of template file name to template
        object.
"""
def gather_templates() -> Mapping[str, Template]:
    env = Environment(
        loader=FileSystemLoader(LAYOUTS_DIR),
        autoescape=select_autoescape(
            enabled_extensions=("html", "jinja"),
        ),
        lstrip_blocks=True,
        trim_blocks=True,
    )

    templates = dict()
    for filename in os.listdir(LAYOUTS_DIR):
        templates[filename] = env.get_template(filename)

    return templates


"""
Step 3. Gather the data from yaml files and put it into one big dictionary
"""
def gather_data() -> Mapping[str, Any]:
    all_data = dict()
    for data_file in os.listdir(DATA_DIR):
        with open(Path(DATA_DIR, data_file), "r") as fd:
            data = yaml.load(fd, Loader=yaml.SafeLoader)
            all_data[Path(data_file).stem] = data

    return all_data


def gather_collections(pages: Sequence[Page]) -> Mapping[str, Sequence[Any]]:
    collections = dict()
    for page in pages:
        for tag in page.tags:
            if tag not in collections:
                collections[tag] = []
            collections[tag].append(page.metadata)
    return collections                


def copy_css():
    css_build_path = Path(BUILD_DIR, CSS_DIR.relative_to(SRC_DIR))
    if css_build_path.exists():
        shutil.rmtree(css_build_path)
    shutil.copytree(CSS_DIR, css_build_path)


def copy_static():
    if not STATIC_DIR.exists():
        return
    build_path = Path(BUILD_DIR, STATIC_DIR.relative_to(SRC_DIR))
    if build_path.exists():
        shutil.rmtree(build_path)
    shutil.copytree(STATIC_DIR, build_path)


def build_site(pages: Sequence[Page], templates, data, collections):
    copy_css()
    copy_static()

    for page in pages:
        template = templates.get(page.layout)
        page_html = template.render(
            **data,
            **page.metadata,
            collections=collections,
            content=page.content,
        )
        build_path = Path(BUILD_DIR, page.get_path_of_index_html())
        write_file(build_path, page_html)
    

def write_file(path: Path, content: str) -> None:
    if not path.parent.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        
    path.touch(exist_ok=True)

    with open(path, "w") as file:
        file.write(content)


def main():
    pages = gather_pages()
    templates = gather_templates()
    data = gather_data()
    collections = gather_collections(pages)
    print(collections)
    build_site(pages, templates, data, collections)

if __name__ == "__main__":
    main()
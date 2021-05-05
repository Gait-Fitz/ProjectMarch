from multiprocessing import Pool
from textwrap import wrap
from datetime import datetime
import sys
import subprocess

copyright = """ This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 Version 3 as published by the Free Software Foundation WITH
 additional terms published by Project MARCH per section 7 of
 the GNU General Public License Version 3.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License INCLUDING the additional terms for
 more details.

 You should have received a copy of the GNU General Public License
 AND the additional terms along with this program. If not,
 see <https://projectmarch.nl/s/LICENSE> and
 <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>."""

class Contribution:
    def __init__(self, author, unix_time):
        self.author = author
        self.year = self.to_commit_year(unix_time)
    def __repr__(self):
        return self.author + " (" + self.year + ")"
    def __eq__(self, other):
        return self.author == other.author and self.year == other.year
    def __hash__(self):
        return hash((self.author, self.year))
    def __lt__(self, other):
        return self.author < other.author 
    def to_commit_year(self,unix_time):
        return datetime.utcfromtimestamp(int(unix_time)).strftime('%Y')

author_map = {
    "Bart van Ingen": "Bart van Ingen",
    "Bart-van-Ingen": "Bart van Ingen",
    "Bas": "Bas Volkers",
    "Bas Volkers": "Bas Volkers",
    "BasVolkers": "Bas Volkers",
    "bentebloks": "Bente Bloks",
    "Bente Bloks": "Bente Bloks",
    "bjornminderman": "Björn Minderman",
    "Björn Minderman": "Björn Minderman",
    "cilia": "Cilia Claij",
    "Ciliaclaij": "Cilia Claij",
    "Cilia Claij": "Cilia Claij",
    "Deakensbubbles": "Rogier Krijnen",
    "Floor": "Floor Heijs",
    "Floor Heijs": "Floor Heijs",
    "gaiavdh": "Gaia van den Heuvel",
    "Gaia van den Heuvel": "Gaia van den Heuvel",
    "Isha": "Isha Dijcks",
    "IshaD": "Isha Dijcks",
    "Isha Dijcks": "Isha Dijks",
    "Isha Dijks": "Isha Dijks",
    "Jitske de Vries": "Jitske de Vries",
    "JitskedeVries": "Jitske de Vries",
    "jorisweeda": "Joris Weeda",
    "JorisWeeda": "Joris Weeda",
    "Joris Weeda": "Joris Weeda",
    "KarlijndeJong": "Karlijn de Jong",
    "Katja Schmahl": "Katja Schmahl",
    "kschmahl": "Katja Schmahl",
    "Maarten ten Voorde": "Maarten ten Voorde",
    "marnixbrands": "Marnix Brands",
    "Marnix Brands": "Marnix Brands",
    "martijnvandermarel": "Martijn van der Marel",
    "Martijn van der Marel": "Martijn van der Marel",
    "Monitor Laptop": "Project MARCH",
    "mtreffers": "Michael Treffers",
    "Michael Treffers": "Michael Treffers",
    "nvanlith": "Niels van Lith",
    "Olav de Haas": "Olav de Haas",
    "Olavhaasie": "Olav de Haas",
    "Project March": "Project MARCH",
    "Project MARCH": "Project MARCH",
    "ReinLukkes": "Rein Lukkes",
    "roel": "Roel Vos",
    "Roel Vos": "Roel Vos",
    "Roelemans": "Roel Vos",
    "rogier": "Rogier Krijnen",
    "Rogier Krijnen": "Rogier Krijnen",
    "Roy Arriens": "Roy Arriëns",
    "Roy Arriëns": "Roy Arriëns",
    "Rutger van Beek": "Rutger van Beek",
    "RutgerVanBeek": "Rutger van Beek",
    "Sophie": "Sophie Bekker",
    "Sophie Bekker": "Sophie Bekker",
    "sopje": "Sophie Bekker",
    "SuperFloortje MegaHeijs": "Floor Heijs",
    "Thijs Raymakers": "Thijs Raymakers",
    "Thijs Veen": "Thijs Veen",
    "tim": "Tim Buckers",
    "Tim": "Tim Buckers",
    "Tim Buckers": "Tim Buckers",
    "TimBuckers": "Tim Buckers",
    "ttveen": "Thijs Veen",
    "wnederpel": "Wolf Nederpel",
    "Wolf Nederpel": "Wolf Nederpel",
}

def find_authors(filename):
    if not should_have_copyright_notice(filename):
        return None

    result = subprocess.run(["git", "blame", "-p", filename], capture_output=True)
    stdout = result.stdout.decode('utf-8')

    authors = set()

    author = ""
    unix_time = ""
    for line in stdout.split('\n'):
        if line.startswith('author '):
            raw_author = author_map[line[7:]]
            try:
                author = author_map[raw_author]
                author = author.replace(" ", "_")
            except KeyError:
                print(f"Author {raw_author} not found")
                author = raw_author
        if line.startswith('author-time '):
            unix_time = line[12:]
        if line.startswith('\t'):
            x = Contribution(author, unix_time)
            authors.add(x)

    if authors == set():
        return None
    else:
        return authors

def group_by_year(authors):
    years = dict()
    for author in authors:
        if author.year not in years:
            years[author.year] = list()
        years[author.year].append(author)
    
    # Sort each year alphabetically
    for year in years:
        years[year] = sorted(years[year])
    return years

def should_have_copyright_notice(filename):
    if "__" in filename:
        return False
    extension = filename.split(".")[-1]
    try:
         _ = prefix_suffix[extension]
         return True
    except KeyError:
        return False

prefix_suffix = {
        "cpp": ("/*\n", " *", "\n*/"),
        "hpp": ("/*\n", " *", "\n*/"),
        "c": ("/*\n", " *", "\n*/"),
        "h": ("/*\n", " *", "\n*/"),
        "py": ("", "#", ""),
        "gait": ("", "#", ""),
        "subgait": ("", "#", ""),
        "srv": ("", "#", ""),
        "msg": ("", "#", ""),
        "test": ("", "#", ""),
        # This is CMake
        "txt": ("#[[\n", "", "\n]]"),
        "cmake": ("", "#", ""),
}
def comment_syntax(filename):
    if "." not in filename:
        return None, None

    extension = filename.split(".")[-1]
    try:
        return prefix_suffix[extension]
    except KeyError:
        return None, None

def generate_copyright_header(filename, authors):
    header = list()
    for year in reversed(sorted(authors)):
        author_list = ", ".join([x.author for x in authors[year]])
        copyright_year = f" Copyright (C) {year}"
        wrapped = f"{copyright_year} {author_list}"
        wrapped = wrap(f"{copyright_year} {author_list}", subsequent_indent="                    ")
        wrapped = [x.replace("_", " ") for x in wrapped]
        header.extend(wrapped)
    header.append("")
    header.append(copyright)
    header = "\n".join(header)

    prefix, middle, suffix = comment_syntax(filename)
    if prefix == None and suffix == None:
        return None

    header = prefix + "\n".join([middle + x for x in header.split('\n')]) + suffix
    header += "\n"
    return header

def add_to_file(file):
    authors = find_authors(file)
    if authors == None:
        return (False, "Authors = None", file)

    authors = group_by_year(authors)
    header = generate_copyright_header(file, authors)
    if header != None:
        print(f"{file}\n{header}")
        data = ""
        try:
            #with open(file, 'r') as original: 
            #    data = original.read()
            #with open(file, 'w') as modified:
            #    modified.write(f"{header}\n{data}")
            return (True, file)
        except:
            return (False, "Opening file failed", file)
    return (False, "Header = None", file)

if __name__ == '__main__':
    # Read filenames from stdin
    # Use something like `find . -type f | python add_license_header.py`
    files = [x.strip() for x in sys.stdin.readlines()]
    with Pool() as pool:
        for i in pool.imap_unordered(add_to_file, files):
            print(i)

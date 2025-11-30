# Data Model: Textbook Content Modules

The data model for this feature is based on the file structure and front matter conventions of the Docusaurus framework, consistent across all content modules.

## Entities

### 1. Module
A self-contained learning unit, represented as a top-level directory within the Docusaurus `docs` folder.
- **Example**: `/docs/module-4-vla-models`

### 2. Page
A single content page within a Module, represented by a Markdown (`.md` or `.mdx`) file.
- **Example**: `intro.md`, `voice-to-action.md`

**Key Attributes (managed via Front Matter):**
- `id`: (string) Unique identifier for the document.
- `title`: (string) The main title of the page, displayed at the top.
- `sidebar_label`: (string) The label used for the page in the navigation sidebar. If not present, `title` is used.
- `sidebar_position`: (number) A number that dictates the order of the page in the sidebar relative to other pages in the same directory.

### 3. Category
A logical grouping of pages, represented by a directory containing a `_category_.json` file. This file controls the properties of the category in the sidebar.
- **Example**: `/docs/module-4-vla-models/_category_.json`

**Key Attributes (in `_category_.json`):**
- `label`: (string) The name of the category in the sidebar.
- `position`: (number) The order of the category in the sidebar.
- `link`: (object) Can be configured to make the category itself a clickable index page.

## Relationships
- A **Module** (directory) contains multiple **Pages** (files) and **Categories** (sub-directories).
- A **Category** contains multiple **Pages**.
- The relationship and hierarchy are defined by the file system structure.

## Example Structure

```text
docs/
└── module-4-vla-models/
    ├── _category_.json
    ├── intro.md
    ├── voice-to-action.md
    ├── cognitive-planning.md
    └── capstone-project.md
```

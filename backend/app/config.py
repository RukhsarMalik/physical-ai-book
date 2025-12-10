import os
from pathlib import Path

# Base path for Docusaurus documentation markdown files
# Assumes the backend is in 'physical_ai_book/backend' and docs are in 'physical_ai_book/physical-ai-book/docs'
# Adjust this path if your directory structure is different
DOCS_BASE_PATH = Path(__file__).parent.parent.parent / "physical-ai-book" / "docs"

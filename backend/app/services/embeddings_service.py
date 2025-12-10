import os
import re
import uuid
from pathlib import Path
from typing import List, Dict, Any

from qdrant_client import models
from openai import OpenAI
from dotenv import load_dotenv
from langchain_text_splitters import MarkdownHeaderTextSplitter

from .qdrant_service import upsert_vectors, COLLECTION_NAME
from .qdrant_service import create_collection_if_not_exists
from ..config import DOCS_BASE_PATH

load_dotenv()

openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


def get_embedding(text: str) -> List[float]:
    """Generate OpenAI embedding."""
    if not text.strip():
        return []
    response = openai_client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding


def extract_metadata_from_path(file_path: Path) -> Dict[str, str]:
    """Extract module + page names."""
    relative_path = file_path.relative_to(DOCS_BASE_PATH)
    parts = relative_path.parts

    module_name = "General"
    page_name = file_path.name

    if len(parts) > 1 and parts[0].startswith("module-"):
        module_name = parts[0].replace("-", " ").replace("module ", "Module ")

    match = re.match(r"(Module \d+) (.*)", module_name)
    if match:
        module_name = f"{match.group(1)}: {match.group(2).title()}"
    else:
        module_name = module_name.title()

    return {
        "module": module_name,
        "page": page_name,
        "source": str(relative_path)
    }


def extract_code_blocks(text: str) -> List[str]:
    return [
        block.strip()
        for block in re.findall(r"```(?:\\w+)?\n(.*?)```", text, re.DOTALL)
    ]


def process_markdown_file(file_path: Path) -> List[Dict[str, Any]]:
    """Split, embed, prepare data for Qdrant."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    markdown_splitter = MarkdownHeaderTextSplitter(
        headers_to_split_on=[
            ("#", "Header1"),
            ("##", "Header2"),
            ("###", "Header3"),
        ]
    )
    md_chunks = markdown_splitter.split_text(content)

    points = []
    file_metadata = extract_metadata_from_path(file_path)

    for i, split in enumerate(md_chunks):
        chunk_content = split.page_content
        metadata = split.metadata

        merged_metadata = {**file_metadata, **metadata}

        title = (
            merged_metadata.get("Header1")
            or merged_metadata.get("Header2")
            or merged_metadata.get("Header3")
            or merged_metadata.get("page")
        )

        embedding = get_embedding(chunk_content)

        if embedding:
            points.append(
                models.PointStruct(
                    id=str(uuid.uuid4()),  # <-- FIXED ID
                    vector=embedding,
                    payload={
                        "text": chunk_content,
                        "module": merged_metadata["module"],
                        "page": merged_metadata["page"],
                        "title": title,
                        "code_blocks": extract_code_blocks(chunk_content),
                        "source": merged_metadata["source"],
                        "chunk_id": i,
                    }
                )
            )

    return points


def generate_embeddings_for_docs(force_refresh: bool = False):
    docs_path = Path(DOCS_BASE_PATH)

    if not docs_path.exists():
        raise FileNotFoundError(f"Docs path not found: {docs_path}")

    all_points = []
    processed_files = 0

    for md_file in docs_path.rglob("*.md"):
        print(f"Processing {md_file.relative_to(docs_path)}...")
        try:
            points = process_markdown_file(md_file)
            all_points.extend(points)
            processed_files += 1
        except Exception as e:
            print(f"Error processing {md_file}: {e}")

    if all_points:
        print(f"Upserting {len(all_points)} points into '{COLLECTION_NAME}'...")
        upsert_vectors(all_points)
        print("Embedding complete!")
    else:
        print("No data to embed.")

    return {"status": "completed", "documents_embedded": processed_files}


if __name__ == "__main__":
    print("Starting embedding generation for Docusaurus docs...")

    # 🔥 Ensures Qdrant collection exists before inserting
    create_collection_if_not_exists()

    try:
        result = generate_embeddings_for_docs(force_refresh=True)
        print(result)
    except Exception as e:
        print(f"Embedding generation failed: {e}")

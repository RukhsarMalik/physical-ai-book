import os
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_URL or not QDRANT_API_KEY:
    raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set.")

client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=60  # Prevent timeout issues
)

COLLECTION_NAME = "book_content"
VECTOR_SIZE = 1536  # model: text-embedding-3-small


# ---------------------------------------------------------
#  CREATE COLLECTION (ONLY IF MISSING)
# ---------------------------------------------------------

def create_collection_if_not_exists():
    """Create Qdrant collection only if it does not already exist."""
    try:
        collections = client.get_collections().collections
        names = [c.name for c in collections]

        if COLLECTION_NAME in names:
            print(f"Collection '{COLLECTION_NAME}' already exists.")
            return True

        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=VECTOR_SIZE,
                distance=models.Distance.COSINE
            ),
        )

        print(f"Collection '{COLLECTION_NAME}' created successfully.")
        return True

    except Exception as e:
        print(f"Error creating collection: {e}")
        return False


# ---------------------------------------------------------
#  UPSERT VECTORS (BATCHED INSERTION)
# ---------------------------------------------------------

def upsert_vectors(points, batch_size=25):
    """Insert vectors into Qdrant in small batches to avoid timeouts."""
    total = len(points)
    print(f"⏳ Upserting {total} vectors in batches of {batch_size}...")

    for i in range(0, total, batch_size):
        batch = points[i:i + batch_size]

        try:
            client.upsert(
                collection_name=COLLECTION_NAME,
                wait=True,
                points=batch,
            )
            print(f"✅ Batch {i//batch_size + 1} inserted ({len(batch)} points)")

        except Exception as e:
            print(f"❌ Error inserting batch {i//batch_size + 1}: {e}")
            raise e


# ---------------------------------------------------------
#  SEARCH VECTORS (USED BY RAG AGENT)
# ---------------------------------------------------------

def search_vectors(query_vector, limit=5, min_score=0.3):
    """Search Qdrant for similar embeddings."""
    try:
        results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=limit,
            score_threshold=min_score,
        )
        return results

    except Exception as e:
        print(f"Error searching vectors: {e}")
        return []


# ---------------------------------------------------------
#  OPTIONAL: GET COLLECTION INFO
# ---------------------------------------------------------

def get_collection_info():
    """Useful for health check endpoint."""
    try:
        info = client.get_collection(collection_name=COLLECTION_NAME)
        return {
            "points_count": info.points_count,
            "status": "ok"
        }
    except Exception as e:
        return {
            "status": "error",
            "message": str(e)
        }

#include "SearchResult.h"

namespace species_id {

bool operator< (const species_id::SearchResult& a,
                const species_id::SearchResult& b) {
  return a.operator<(b);
}


}

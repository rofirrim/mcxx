#include "hash.h"

void
hash_init (Hash * h, int size, HASH_FUNC * hash_func,
	   KEYCMP_FUNC * keycmp_func)
{
  int i;
  h->size = size;
  h->hash_func = hash_func;
  h->keycmp_func = keycmp_func;
  h->table = NEW_ARRAY (HashNode *, size);
  for (i = 0; i < size; i++)
    h->table[i] = NULL;
  h->no_items = 0;
  h->free_keys = true;
  h->free_items = false;
  h->df_keys = NULL;
  h->df_items = NULL;
}

void
hash_set (Hash * h, int op, void *value)
{
  switch (op)
    {
    case HASH_FREE_KEYS:
      h->free_keys = (bool_type) value;
      break;
    case HASH_FREE_ITEMS:
      h->free_items = (bool_type) value;
      break;
    case HASH_KEYS_DESTRUCTOR:
      h->df_keys = value;
      break;
    case HASH_ITEMS_DESTRUCTOR:
      h->df_items = value;
      break;
    }
}

Hash *
hash_create (int size, HASH_FUNC * hash_func, KEYCMP_FUNC * keycmp_func)
{
  Hash *new_hash;

  new_hash = NEW (Hash);
  hash_init (new_hash, size, hash_func, keycmp_func);

  return new_hash;
}

void
hash_dump (Hash * h)
{
  int i;
  HashNode *node, *node_next;

  for (i = 0; i < h->size; i++)
    for (node = h->table[i]; node; node = node_next)
      {
	node_next = node->next;
	if (h->free_items)
	  {
	    if (h->df_items)
	      (*h->df_items) (node->item);
	    FREE (node->item);
	  }
	if (h->free_keys)
	  {
	    if (h->df_keys)
	      (*h->df_keys) (node->key);
	    FREE (node->key);
	  }
	FREE (node);
      }

  FREE (h->table);
  h->size = 0;
  h->no_items = 0;
}

void
hash_destroy (Hash * h)
{
  if (!h)
    return;

  hash_dump (h);
  FREE (h);
}

void
hash_put (Hash * h, void *key, void *item)
{
  int i;
  HashNode *node;

  if (!h)
    return;

  i = (*h->hash_func) (key, h->size);

  node = NEW (HashNode);
  node->key = key;
  node->item = item;
  node->prev = NULL;

  if (h->table[i])
    h->table[i]->prev = node;
  node->next = h->table[i];
  h->table[i] = node;
  h->no_items++;
}

HashNode *
hash_getnode (Hash * h, int i, void *key)
{
  HashNode *node;

  if (i < 0 || i >= h->size)
    return NULL;

  for (node = h->table[i]; node; node = node->next)
    if (!(*h->keycmp_func) (key, node->key))
      return node;
  return NULL;
}

void *
hash_get (Hash * h, void *key)
{
  HashNode *node;

  if (!h)
    return NULL;
  node = hash_getnode (h, (*h->hash_func) (key, h->size), key);
  if (!node)
    return NULL;

  return node->item;
}

void *
hash_delete (Hash * h, void *key)
{
  HashNode *node;
  void *item;
  int i;

  if (!h)
    return NULL;
  i = (*h->hash_func) (key, h->size);
  node = hash_getnode (h, i, key);
  if (!node)
    return NULL;

  if (node->next)
    node->next->prev = node->prev;
  if (node->prev)
    node->prev->next = node->next;

  if (h->table[i] == node)
    h->table[i] = node->next;

  if (h->free_items)
    {
      if (h->df_items)
	(*h->df_items) (node->item);
      FREE (node->item);
      item = NULL;
    }
  else
    item = node->item;

  if (h->free_keys)
    {
      if (h->df_keys)
	(*h->df_keys) (node->key);
      FREE (node->key);
    }
  FREE (node);

  h->no_items--;

  return item;
}

/* common hash functions */

#include <ctype.h>
#include <string.h>

int
hash_string (char *name, int size)
{
  if (!name || name[0] == '\0')
    return 0;
  if (strlen (name) < 3)
    return name[0] % size;

  return (name[0] + name[1] + name[3]) % size;
}


int
hash_caseless_string (char *name, int size)
{
  if (!name || name[0] == '\0')
    return 0;

  return (tolower (name[0]) + tolower (name[1]) + tolower (name[3])) % size;
}

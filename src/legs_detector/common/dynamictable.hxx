/***************************************************************************
                          dynamictable.hxx  -  description
                             -------------------
    begin                : Fri Dec 7 2001
    copyright            : (W) 2001 by Cyrill Stachniss
    email                :
 ***************************************************************************/

#include <stdio.h>

template<class T>
DynamicTable<T>::DynamicTable() {
	DynamicTable(700);
}

template<class T>
DynamicTable<T>::DynamicTable(int s) {
  size=s;
  setAutoDelete(false);

  if ( s < 1) {
    size = 6;
    fprintf(stderr, "%s at line %d : error in table, initial size s=%i! setting size=%i\n", __FILE__,__LINE__,s,size);
  }

  // size should be even
  if ((size % 2) == 1)
    size++;

  nextField = 0;
  fields = new T*[size]; /* check_alloc checked */
  if (fields == NULL) {
    fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__, size);
    exit(0);
  }

  // by Oscar
  for(int n=0; n < size; n++) {
  	fields[n] = NULL;
  }
}



template<class T>
DynamicTable<T>::DynamicTable(DynamicTable<T>& tab, int desired_size) {

  setAutoDelete(tab .autoDelete);

  if (desired_size > tab.nextField) {
    size = desired_size;
    nextField = tab.nextField;
  }
  else {
    size = tab.size;
    nextField = tab.nextField;
  }

  fields = new T*[size]; /* check_alloc checked */
  if (fields == NULL) {
    fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__, size);
    exit(0);
  }

  for (int i=0; i < nextField; i++) {
    fields[i] = tab.fields[i];
  }

  if (nextField < size)
    fields[nextField] = NULL;
}



template<class T>
DynamicTable<T>::~DynamicTable() {
  if (autoDelete)    {
    for(int n=0; n < numberOfElements(); n++) {
      if (fields[n] != NULL) {
	delete fields[n];
	fields[n] = NULL;
      }
    }
  }

  delete [] fields;
}


template<class T>
void DynamicTable<T>::add(DynamicTable<T>& otherTab) {

  int needMore =  otherTab.numberOfElements() -  (size - nextField) + 1;

  if (needMore > 0) {
    T** fields2 = new T*[2*size+needMore]; /* check_alloc checked */
    if (fields2 == NULL) {
      fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__, 2*size+needMore);
      exit(0);
    }
   
    for (int i = 0; i < nextField; i++)
      fields2[i] = fields[i]; 	
    size = 2*size+needMore;	
    delete [] fields;
    fields = fields2;
  }

  int othersize = otherTab.numberOfElements();
  for (int i = 0; i < othersize; i++) {
    fields[nextField++] = otherTab.fields[i];
  }
  fields[nextField] = NULL;
}

/** F�gt ein Feld an */
template<class T>
void DynamicTable<T>::add(T* element) {
  if (nextField == size)    {
    T** fields2 = new T*[2*size];/* check_alloc checked */
    if (fields2 == NULL) {
      fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__, 2*size);
      exit(0);
    }
    for (int i = 0; i < nextField; i++)
      fields2[i] = fields[i]; 	
    size *= 2;	
    delete [] fields;
    fields = fields2;
  }
  
  fields[nextField++] = element;
  fields[nextField] = NULL;
}

/** Liefert das Field-Array selbst */
template<class T>
T** DynamicTable<T>::getElements() {
  return fields;
}


/** Liefert Zeiger auf das i-te Element */
template<class T>
T* DynamicTable<T>::getElement(int i) {
  if (i >= nextField)
    return NULL;
  else
    return fields[i];
}

/** Liefert Zeiger auf das 1-te Element */
template<class T>
T* DynamicTable<T>::getFirst() {
  if (nextField <= 0)
    return NULL;
  else
    return fields[0];
}


/** Liefert Zeiger auf das letzte Element */
template<class T>
T* DynamicTable<T>::getLast() {
  if (nextField <= 0)
    return NULL;
  else
    return fields[nextField-1];
}


template<class T>
int DynamicTable<T>::numberOfElements() {
  return nextField;
}

template<class T>
int DynamicTable<T>::num() {
  return nextField;
}

template<class T>
int DynamicTable<T>::allocated() {
  return size;
}



template<class T>
void DynamicTable<T>::reverse() {
  if (nextField <= 1)
    return;
	
  T* tmp = NULL;
	
  for (int i=0; i < nextField / 2; i++) {
    tmp = fields[i];
    fields[i] = fields[nextField-1-i];
    fields[nextField-1-i] = tmp;
  }
}

/** sets the autodelete variable -> see autoDelete in this class */
template<class T>
void DynamicTable<T>::setAutoDelete(bool del) {
  autoDelete = del;
}

template<class T>
void DynamicTable<T>::removeLast() {
  
  if (nextField == 0)
    return;

  nextField--;
  if (autoDelete) 
    delete fields[nextField];
  fields[nextField] = NULL;
}



template<class T>
void DynamicTable<T>::remove_unconsolidated(int i) {
  if (i < 0 || i >= nextField || nextField <= 0)
    return;
  if (autoDelete && fields[i] != NULL) 
    delete fields[i];
  fields[i] = NULL;
}

template<class T>
void DynamicTable<T>::consolidate() {

  int i=0,j=0;
  while(i < nextField) {
    if (fields[i] == NULL) {
      while (fields[j] == NULL && j < nextField)
	j++;
      if (j == nextField) {
	nextField = i;
	return;
      }
      fields[i] = fields[j];
      fields[j] = NULL;
    }
    j++;
    i++;
  }

}



template<class T>
void DynamicTable<T>::remove(int i) {

  if ((i < nextField) && (nextField > 0) && (i>=0)) {
    if (autoDelete)
      delete fields[i];
    
    fields[i] = fields[nextField-1];
    fields[nextField-1] = NULL;
    nextField--;
  }
}

template<class T>
void DynamicTable<T>::removeInOrder(int i) {

  if ((i < nextField) && (nextField > 0) && (i>=0)) {
    if (autoDelete)
      delete fields[i];
    for (int j = i; j < nextField-1; j++)
      fields[j] = fields[j+1];
    fields[nextField-1] =  NULL;
    nextField--;
  }
}



template<class T>
void DynamicTable<T>::remove(T* element) {
  
  for(int i = 0; i < nextField; i++) {
    if (fields[i] == element) {
      if (autoDelete)
	delete fields[i];
      fields[i] = fields[nextField-1];
      fields[nextField-1] = NULL;
      nextField--;
    }
  }
}

template<class T>
void DynamicTable<T>::clear() {

  if (autoDelete) {
    for(int i = 0; i < nextField; i++) 
      delete fields[i];
  }
  nextField = 0;
  fields[0] = NULL;
}


template<class T> 
DynamicTable<T>*  DynamicTable<T>::getIntersection(DynamicTable<T>* other) {
  
  DynamicTable<T>* intersection = new DynamicTable<T>(size+1); /* check_alloc checked */
  if (intersection == NULL) {
    fprintf(stderr,"out of memory! %s at line %d\n", __FILE__,__LINE__);
    exit(0);
  }
  
  T** elemOther = other->getElements();
  
  for (int i = 0; i < other->numberOfElements(); i++) {
    int j = 0;
    bool abort = false;
    while (!abort && j < numberOfElements()) {
      
      if (elemOther[i] == fields[j]) {
	//	fprintf(stderr,"inter(%d %d)->%p\n",i,j,fields[j]);
	intersection->add(fields[j]);
	abort = true;
      }
      else 
	j++;
    }
  }
  return intersection;
}

template<class T> 
bool DynamicTable<T>::contains(T* element) {
  
  for (int i = 0; i < numberOfElements(); i++) {  
    if (element == fields[i])
      return true;
  }
  return false;
}


template<class T> 
T* DynamicTable<T>::replace(int idx, T* elem) {
  T* replaced = NULL;
  if (idx >= 0 && idx < nextField) {
    replaced = fields[idx];
    fields[idx] = elem;
  }
  return replaced;
}

template<class T> 
void DynamicTable<T>::swap(int idx1, int idx2) {
  if (idx1 >= 0 && idx1 < nextField && idx2 >= 0 && idx2 < nextField) {
    T* tmp = fields[idx1];
    fields[idx1] = fields[idx2];
    fields[idx2] = tmp;
  }
}


//////////////////////////////////////////////////////////////



template<class T>
PrimitiveDynamicTable<T>::PrimitiveDynamicTable(int s, T null_elem) {
  null_element = null_elem;
  init(s);
}

template<class T>
PrimitiveDynamicTable<T>::PrimitiveDynamicTable(int s) {

  T null_elem;
  unsigned char* p =  (unsigned char*) (&null_elem);
  for (unsigned int i = 0; i < sizeof(T); i++) 
    p[i] = 0;
  
  init(s);
}

template<class T>
void PrimitiveDynamicTable<T>::init(int s) {

  if ( s < 1)  {
    this->size = 6;
    fprintf(stderr, "%s at line %d : error in table, initial size s=%i! setting size=%i\n", __FILE__,__LINE__,s,size);
  }
  else
    this->size = s;
  
  // size should be even
  if ((this->size % 2) == 1)
    this->size++;

  //  fprintf(stderr, "%s: %s at line %d : size = %d!\n", __FILE__,__FUNCTION__,__LINE__,size);

  nextField = 0;
  fields = new T[size];  /* check_alloc checked */
  if (fields == NULL) {
    fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__, size);
    exit(0);
  }
  fields[0] = null_element;
}



template<class T>
PrimitiveDynamicTable<T>::~PrimitiveDynamicTable() {
  delete [] fields;
}

template<class T>
void PrimitiveDynamicTable<T>::add(PrimitiveDynamicTable<T>& otherTab) {

  int needMore =  otherTab.numberOfElements() -  (size - nextField) + 1;

  if (needMore > 0) {
    T* fields2 = new T[2*size+needMore];  /* check_alloc checked */
    if (fields2 == NULL) {
      fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__, 2*size+needMore);
      exit(0);
    }

    for (int i = 0; i < nextField; i++)
      fields2[i] = fields[i]; 	
    size = 2*size+needMore;	
    delete [] fields;
    fields = fields2;
  }

  int othersize = otherTab.numberOfElements();
  for (int i = 0; i < othersize; i++) {
    fields[nextField++] = otherTab.fields[i];
  }
 fields[nextField] = null_element;
}

/** F�gt ein Feld an */
template<class T>
void PrimitiveDynamicTable<T>::add(T element)
{
  if (nextField == size)    {
    T* fields2 = new T[2*size];  /* check_alloc checked */
    if (fields2 == NULL) {
      fprintf(stderr,"out of memory! %s at line %d, size=%d\n", __FILE__,__LINE__, 2*size);
      exit(0);
    }

    for (int i = 0; i < nextField; i++)
      fields2[i] = fields[i]; 	
    size *= 2;	
    delete [] fields;
    fields = fields2;
  }

  fields[nextField++] = element;
  fields[nextField] =  null_element;
}

/** Liefert das Field-Array selbst */
template<class T>
T* PrimitiveDynamicTable<T>::getElements() {
  return fields;
}

/** Liefert das i-te Element */
template<class T>
T PrimitiveDynamicTable<T>::getElement(int i) {
  if (i >= nextField)
    return null_element;
  else
    return fields[i];
}

/** Liefert  das 1-te Element */
template<class T>
T PrimitiveDynamicTable<T>::getFirst() {
  if (nextField <= 0)
    return null_element;
  else
    return fields[0];
}


/** Liefert  das letzte Element */
template<class T>
T PrimitiveDynamicTable<T>::getLast() {
  if (nextField <= 0)
    return  null_element;
  else
    return fields[nextField-1];
}


template<class T>
int PrimitiveDynamicTable<T>::numberOfElements() {
  return nextField;
}

template<class T>
int PrimitiveDynamicTable<T>::num() {
  return nextField;
}

template<class T>
int PrimitiveDynamicTable<T>::allocated() {
  return size;
}


template<class T>
void PrimitiveDynamicTable<T>::reverse() {
  if (nextField <= 1)
    return;
  
  T tmp = NULL;
	
  for (int i=0; i < nextField / 2; i++) {
    tmp = fields[i];
    fields[i] = fields[nextField-1-i];
    fields[nextField-1-i] = tmp;
  }
}


template<class T>
void PrimitiveDynamicTable<T>::remove(int i) {

  if ((i < nextField) && (nextField > 0) && (i>=0)) {
    fields[i] = fields[nextField-1];
    fields[nextField-1] =  null_element;
    nextField--;
  }
}

template<class T>
void PrimitiveDynamicTable<T>::removeInOrder(int i) {

  if ((i < nextField) && (nextField > 0) && (i>=0)) {
    for (int j = i; j < nextField-1; j++)
      fields[j] = fields[j+1];
    fields[nextField-1] =  null_element;
    nextField--;
  }
}


template<class T>
void PrimitiveDynamicTable<T>::removeByValue(T element) {

  for(int i = 0; i < nextField; i++) {
    if (fields[i] == element) {
      fields[i] = fields[nextField-1];
      fields[nextField-1] = NULL;
      nextField--;
    }
  }
}

template<class T>
void PrimitiveDynamicTable<T>::clear() {
  nextField = 0;
  fields[0] = null_element;
}


template<class T> 
PrimitiveDynamicTable<T>*  PrimitiveDynamicTable<T>::getIntersection(PrimitiveDynamicTable<T>* other) {

  PrimitiveDynamicTable<T>* intersection = new PrimitiveDynamicTable<T>(size+1);  /* check_alloc checked */
  if (intersection == NULL) {
    fprintf(stderr,"out of memory! %s at line %d\n", __FILE__,__LINE__);
    exit(0);
  }

  T* elemOther = other->getElements();

  for (int i = 0; i < other->numberOfElements(); i++) {
    int j = 0;
    bool abort = false;
    while (!abort && j < numberOfElements()) {
      
      if (elemOther[i] == fields[j]) {
	intersection->add(fields[j]);
	abort = true;
      }
      else 
	j++;
    }
  }
  return intersection;
}

template<class T> 
bool PrimitiveDynamicTable<T>::contains(T element) {
  
  for (int i = 0; i < numberOfElements(); i++) {  
    if (element == fields[i])
      return true;
  }
  return false;
}


template<class T> 
T PrimitiveDynamicTable<T>::replace(int idx, T elem) {
  T replaced = null_element;
  if (idx >= 0 && idx < nextField) {
    replaced = fields[idx];
    fields[idx] = elem;
  }
  return replaced;
}


template<class T> 
void PrimitiveDynamicTable<T>::swap(int idx1, int idx2) {
  if (idx1 >= 0 && idx1 < nextField && idx2 >= 0 && idx2 < nextField) {
    T tmp = fields[idx1];
    fields[idx1] = fields[idx2];
    fields[idx2] = tmp;
  }
}

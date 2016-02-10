/***************************************************************************
                          dynamictable.h  -  description
                             -------------------
    begin                : Fri Dec 7 2001
    copyright            : (W) 2001 by Cyrill Stachniss
    email                : 
 ***************************************************************************/

#ifndef DYNAMICTABLE_H
#define DYNAMICTABLE_H

#include <stdlib.h>

/** a dynnamic array of pointer to elements of type T 
  * witch grows automaticly.
  *@author Cyrill Stachniss
  */
template<class T>
class DynamicTable {
public: 

  // by Oscar
  /** O(1) */
  DynamicTable(); // creates a list of 700 elements 

  /** O(1) */
  DynamicTable(int s);

  /** O(n)  */
  DynamicTable(DynamicTable<T>& tab, int desired_size=-1);

  /** O(1) if autodelete==false, else O(n) */
  ~DynamicTable();

  /** sets the autodelete variable -> see autoDelete in this class , O(1)*/
  void setAutoDelete(bool del);

  /** O(1) */
  int numberOfElements();
  /** O(1), does the same as numberOfElements */
  int num();

  /** O(1), allocated memory (size of array) */
  int allocated();

  /** O(n) */
  bool contains(T* element);

  /** O(1) */
  T* operator[](int idx) { return getElement(idx);}
  /** O(1) */
  T* getElement(int i);
  /** O(1) */
  T* getFirst();
  /** O(1) */
  T* getLast();
  /** O(1) */
  T** getElements();
  
  /** O(1) */
  void add(T* element);
  /** O(n + nOther) */
  void add(DynamicTable<T>& otherTab);

  /** O(1) */
  T* replace(int idx, T* elem);
  /** O(1) */
  void swap(int idx1, int idx2);


  /**  O(1), BUT DISTRUBS THE TABLE, CALL CONSOLIDATE AFTER THIS!!!!!!!!*/
  void remove_unconsolidated(int i);
  /*** O(n) **/
  void consolidate();


  /** O(1) */
  void removeLast();
  /** Destroys the order ,O(1) */
  void remove(int i);
  /** O(n) */
  void remove(T* element);
  /** O(n) */
  void removeByValue(T* element) { remove(element); }
  /** maintais the order, O(n) */
  void removeInOrder(int i);

  /** O(1) if autodelete==false, else O(n) */
  void clear();

  /** O(n) */
  void reverse();

  /** O(n + nOther) */
  DynamicTable<T>* getIntersection(DynamicTable<T>* other);


protected:
  /** Array von Zeigern auf die einzelnen fields-Elemente */
  T** fields;
  int size;
  int nextField;
		
  /** delete the elements in the table during destruction */
  bool autoDelete;
};

/** a dynnamic array of elements of type T  (no pointers!)
  * witch grows automaticly.
  *@author Cyrill Stachniss
  */
template<class T>
class PrimitiveDynamicTable {
public: 
  /** O(1) */
  PrimitiveDynamicTable(int s);
  /** O(1) */
  PrimitiveDynamicTable(int s, T null_elem);
  /** O(1) */
  ~PrimitiveDynamicTable();

  /** O(1) */
  int numberOfElements();
  /** O(1), does the same as numberOfElements */
  int num();

  /** O(1), allocated memory (size of array) */
  int allocated();

  /** O(n) */
  bool contains(T element);

  /** O(1) */
  T operator[](int idx) { return getElement(idx);}
  /** O(1) */
  T getElement(int i);
  /** O(1) */
  T getFirst();
  /** O(1) */
  T getLast();
  /** O(1) */
  T* getElements();

  /** O(1) */
  void add(T element);
  /** O(n + nOther) */
  void add(PrimitiveDynamicTable<T>& otherTab);

  /** O(1) */
  T replace(int idx, T elem);
  /** O(1) */
  void swap(int idx1, int idx2);

  /** maintais the order O(n) */
  void removeInOrder(int i);
  /** Destroys the order, O(1) */
  void remove(int i);
  /** O(n) */
  void removeByValue(T element);
  /** O(1) */
  void clear();

  /** O(n) */
  void reverse();

  /** O(n + nOther) */
  PrimitiveDynamicTable<T>* getIntersection(PrimitiveDynamicTable<T>* other);
	
protected:
  void init(int s);
  /** Array der einzelnen fields-Elemente */
  T* fields;
  int size;
  int nextField;		

  T null_element;  // null-element korrespondiert zu NULL-ptr

};

typedef PrimitiveDynamicTable<int>   PDynTabInt;
typedef PrimitiveDynamicTable<float> PDynTabFloat;
typedef PrimitiveDynamicTable<double> PDynTabDouble;

#include "dynamictable.hxx"

#endif

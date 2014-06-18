
#ifndef _Tribots_DynamicRingBuffer_h_
#define _Tribots_DynamicRingBuffer_h_

#include <stdexcept>
#include <vector>

namespace Tribots {

  /** Klasse DynamicRingBuffer realisiert einen Ringspeicher dynamisch 
      veraenderbarer Groesse; Zugriff erfolgt relativ zu einer Ankerposition.

      Interner Aufbau: es handelt sich um eine einfach verkettete Liste mit
      Ringschluss. Ein Anker erlaubt den Zugriff auf die Elemente. Der Ringpuffer
      enthaelt intern stets mindestens ein Element, selbst wenn die Groesse
      mit 0 (leer) angegeben wird. Dies verhindert Speicherfehler beim
      Zugriff auf einen leeren Ringpuffer. Entfernte Ringpufferelemente
      werden in einer einfach verketteten Liste empty_nodes aufbewahrt
      und bei Bedarf spaeter wieder in den Ringpuffer eingefuegt. */
  template<class T> class DynamicRingBuffer {
    /** Interne Knotenstruktur des Ringspeichers */
    struct Node {
      T elem;
      Node* next;
    };
  private:
    unsigned int cur_size;       ///< aktuelle Groesse

    Node* anchor;                ///< Verankerung
    Node* pred_anchor;           ///< Vorgaengerknoten zur Verankerung
    Node* empty_nodes;           ///< Liste der leeren Knoten, die zur Wiederverwendung aufbewahrt werden

  public:
    /** Konstruktor; arg1= Speichergroesse */
    DynamicRingBuffer (unsigned int =1) throw (std::bad_alloc);
    /** Copy-Konstruktor */
    DynamicRingBuffer (const DynamicRingBuffer<T>&) throw (std::bad_alloc);
    /** Destruktor */
    ~DynamicRingBuffer () throw ();
    /** Zuweisung */
    const DynamicRingBuffer<T>& operator= (const DynamicRingBuffer<T>&) throw (std::bad_alloc);

    /** liefert die momentane Groesse des Speichers */
    unsigned int size () const throw ();
    /** Weiterbewegen der Verankerung um arg1 Elemente; negative Werte moeglich, aber ineffizient */
    void step (int =1) throw ();
    /** liefert das Element an Position Anker+arg1; negative Werte moeglich, aber ineffizient */
    const T& get (int =0) const throw ();
    /** setzt das Element an Position Anker+arg1; wirft selbst keine Ausnahmen */
    void set (const T&, unsigned int =0);
    /** fuegt ein Element vor der Ankerposition ein, Anker steht danach auf neuem Element;
        get(1) liefert das ehemalige Ankerelement.  wirft ggf. eine bad_alloc Ausnahme */
    void insert (const T&);
    /** Element an der Ankerposition entfernen, wenn size()>0.
        Anker steht danach auf Element Anker+1 */
    void erase () throw ();
    /** Puffergroesse auf n Elemente setzen. Betsehende Inhalte gehen dabei verloren. wirft ggf. eine
        bad_alloc Ausnahme */
    void resize (unsigned int n);
    /** Puffer vollstaendig leeren */
    void clear () throw ();
    /** Puffer in einen vector schreiben. Wirft ggf. bad_alloc Ausnahme */
    void toVector (std::vector<T>& dest) const;
  };

}








// Implementierung, wegen template-Deklaration:

template<class T> Tribots::DynamicRingBuffer<T>::DynamicRingBuffer (unsigned int n) throw (std::bad_alloc) : empty_nodes(NULL) {
  cur_size = n;
  if (n==0) 
    n=1;
  anchor = new Node;
  pred_anchor=anchor;
  while ((--n)>0) {
    Node* new_node = new Node;
    new_node->next=anchor;
    anchor=new_node;
  }
  pred_anchor->next=anchor;
}

template<class T> Tribots::DynamicRingBuffer<T>::~DynamicRingBuffer () throw () {
  pred_anchor->next=NULL;
  while (anchor->next) {
    Node* new_node = anchor;
    anchor = anchor->next;
    delete new_node;
  }
  while (empty_nodes) {
    Node* new_node = empty_nodes;
    empty_nodes=empty_nodes->next;
    delete new_node;
  }
}

template<class T> Tribots::DynamicRingBuffer<T>::DynamicRingBuffer (const Tribots::DynamicRingBuffer<T>& arg) throw (std::bad_alloc) : empty_nodes (NULL) {
  cur_size = 1;
  anchor = new Node;
  pred_anchor=anchor;
  anchor->next=anchor;

  operator= (arg);
}

template<class T> const Tribots::DynamicRingBuffer<T>& Tribots::DynamicRingBuffer<T>::operator= (const Tribots::DynamicRingBuffer<T>& arg) throw (std::bad_alloc) {
  // ueberzaehlige Knoten entfernen
  const unsigned int sz=arg.size();
  while (size()>sz)
    erase ();
  // kopieren bzw. Knoten einfuegen
  for (unsigned int i=0; i<sz; i++) {
    if (size()<sz)
      insert (arg.get (i));
    else
      set (arg.get(i));
    step ();
  }
  return (*this);
}

template<class T> unsigned int Tribots::DynamicRingBuffer<T>::size () const throw () {
  return cur_size;
}

template<class T> void Tribots::DynamicRingBuffer<T>::step (int n) throw () {
  if (cur_size==0)
    return;
  while (n<0)
    n+=static_cast<int>(cur_size);
  while (n-->0)
    pred_anchor=pred_anchor->next;
  anchor=pred_anchor->next;
}

template<class T> const T&  Tribots::DynamicRingBuffer<T>::get (int n) const throw () {
  if (cur_size==0)
    return anchor->elem;
  while (n<0)
    n+=static_cast<int>(cur_size);
  Node* ptr = anchor;
  while ((n--)>0)
    ptr=ptr->next;
  return ptr->elem;
}

template<class T> void Tribots::DynamicRingBuffer<T>::set (const T& e, unsigned int n) {
  if (cur_size==0)
    return;
  Node* ptr = anchor;
  while ((n--)>0)
    ptr=ptr->next;
  ptr->elem = e;
}

template<class T> void Tribots::DynamicRingBuffer<T>::insert (const T& e) {
  if (cur_size==0) {
    cur_size=1;
    set (e);
    return;
  } else if (empty_nodes) {
    pred_anchor->next=empty_nodes;
    empty_nodes=empty_nodes->next;
  } else
    pred_anchor->next=new Node;   // hier wird ggf. bad_alloc geworfen
  cur_size++;
  pred_anchor->next->next = anchor;
  pred_anchor->next->elem = e;
  anchor=pred_anchor->next;
}

template<class T> void Tribots::DynamicRingBuffer<T>::erase () throw () {
  if (cur_size>1) {
    cur_size--;
    pred_anchor->next=anchor->next;
    anchor->next=empty_nodes;
    empty_nodes=anchor;
    anchor=pred_anchor->next;
  } else {
    cur_size=0;
  }
}

template<class T> void Tribots::DynamicRingBuffer<T>::resize (unsigned int n) {
  while (n<size())
    erase();
  if (n>size()) {
    T defaultelem;
    while (n>size())
      insert (defaultelem);
  }
}

template<class T> void Tribots::DynamicRingBuffer<T>::clear () throw () {
  while (size()>0)
    erase();
}

template<class T> void Tribots::DynamicRingBuffer<T>::toVector (std::vector<T>& dest) const {
  dest.resize (size());
  for (unsigned int i=0; i<size(); i++)
    dest[i]=get(i);
}

#endif

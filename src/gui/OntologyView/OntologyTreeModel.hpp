#ifndef TEMPL_GUI_ONTOLOGY_VIEW_ONTOLOGY_TREE_MODEL_HPP
#define TEMPL_GUI_ONTOLOGY_VIEW_ONTOLOGY_TREE_MODEL_HPP

#include <QAbstractItemModel>
#include <owlapi/OWLApi.hpp>

namespace templ {
namespace gui {

class OntologyTreeModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    explicit OntologyTreeModel(const owlapi::model::OWLOntology::Ptr& ontology,
            const owlapi::model::IRI& iri,
            QObject* parent = 0);
    virtual ~OntologyTreeModel();

//    QVariant data(const QModelIndex& index, int role) const Q_DECL_OVERRIDE;
//
//
    virtual QVariant data(const QModelIndex &index, int role) const;
//    Qt::ItemFlags flags(const QModelIndex &index) const Q_DECL_OVERRIDE;
//    QVariant headerData(int section, Qt::Orientation orientation,
//                        int role = Qt::DisplayRole) const Q_DECL_OVERRIDE;
    virtual QModelIndex index(int row, int column,
                      const QModelIndex &parent = QModelIndex()) const;
    virtual QModelIndex parent(const QModelIndex &index) const;
    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;

private:
//    void setupModelData(const QStringList& lines, TreeItem *parent);
//
//    TreeItem* rootItem;
    owlapi::model::IRIList mIris;
};

} // end namespace gui
} // end namespace templ
#endif // TEMPL_GUI_ONTOLOGY_VIEW_ONTOLOGY_TREE_MODEL_HPP

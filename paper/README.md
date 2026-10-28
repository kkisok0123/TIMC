# 修订版论文（main_timc_revised）

投稿 TIMC 返修稿的 PDF、LaTeX 源码与图。30 页，9 图 7 表。

## 与代码版本的对应关系

论文中的全部数值取自 `revision` 分支提交 `9339ec5` 的实现与
`revision_runs/` 下的 58 次仿真（消融 8、失配 18、阻力 12、随机 20）。
`df9f724` 的 revert 已把工作树恢复到与 `9339ec5` 完全一致，因此论文与当前
分支代码一一对应；`2026-10-25` 早间的 RMS 对齐改动未保留，论文引用的仍是
随机扰动平移 RMS = 1.0 m/s²、周期扰动约 2.15 m/s² 的那组数据。

## PDF 来源

`main_timc_revised.pdf` 即 `main_timc_revised.tex` 两遍 pdfLaTeX 的输出
（MiKTeX pdfTeX-1.40.28）。复核时用同一份源码重编，得到 1,430,622 B 的 PDF，
与 `paper_polished/main_timc_revised_new.pdf` 只有 74 个字节不同（`/CreationDate`、
`/ID`），抽取文本逐字一致（67,472 字符）。

## 编译

```bash
pdflatex main_timc_revised
pdflatex main_timc_revised
```

需要 `sagej.cls`（同目录）与 `figures/`（`\graphicspath{{figures/}}`）。
Overleaf 上传本目录即可。
